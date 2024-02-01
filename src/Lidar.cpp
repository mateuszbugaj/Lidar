#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "semphr.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/cm3/nvic.h>

#include "Controler.hpp"

extern "C"{
	#include "uart_lib.h"
	#include "i2c.h"
}

/*
Pinout:
- PC13 - On-board diode
- PA9  - USB-UART RX
- PA10 - USB-UART TX

- PB12 - Alpha motor step
- PB13 - Alpha motor dir
- PB14 - Beta motor step
- PB15 - Beta motor dir

- PB6  - I2C1 SCL
- PB7  - I2C1 SDA
- PB10 - I2C2 SCL
- PB11 - I2C2 SDA

- PA0  - Main task indicator
- PA1  - Stepper motor task indicator
- PA2  - Rotation sensor task indicator
*/

#define DIODE_GPIO GPIO13

// PORT A
#define IND_PORT GPIOA
#define MAIN_TASK_IND GPIO0
#define STEPPER_MOTOR_TASK_IND GPIO1
#define ROTATION_SENSOR_TASK_IND GPIO2
#define INPUT_BUTTON GPIO7
#define STEPPER_MOTOR_SLEEP GPIO11

#define COIL_NUMBER 4
#define AS5600_ADDR 0x36
#define AS5600_STATUS 0x0B
#define AS5600_ANGLE_L 0x0C
#define AS5600_ANGLE_H 0x0D

#define ALPHA_OFFSET 232
#define BETA_OFFSET 30

#define MICRO_STEPPING 4
#define GEAR_RATIO 0.38

struct Pin {
	uint32_t gpioport;
	uint16_t gpios;
};

struct StepperMotor {
	Pin step;
	Pin dir;
	int stepsToMove;
};

struct Point {
	int16_t alphaDegrees;
	int16_t betaDegrees;
};

struct ReadingTaskData {
	uint32_t i2c;
	SemaphoreHandle_t* signal;
	int16_t* valueToUpdate;
};

struct StepperMotorTaskData {
	StepperMotor* motorAlpha;
	StepperMotor* motorBeta;
};

static ReadingTaskData 
	alphaReadingTaskData, 
	betaReadingTaskData,
	alphaReadingStatusTaskData,
	betaReadingStatusTaskData;

static StepperMotor motorAlpha, motorBeta;
static StepperMotorTaskData stepperMotorTaskData;
static SemaphoreHandle_t 
	readAlphaPositionSignal, 
	readBetaPositionSignal, 
	readAlphaSensorStatusSignal, 
	readBetaSensorStatusSignal;

static QueueHandle_t movementQueue;
static Point currentPosition;
static int16_t currentAlphaSensorStatus, currentBetaSensorStatus;

void stepperMotorInit(StepperMotor& sm){
	gpio_set_mode(
		sm.step.gpioport,
		GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		sm.step.gpios
	);

	gpio_set_mode(
		sm.dir.gpioport,
		GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		sm.dir.gpios
	);

	gpio_clear(sm.step.gpioport, sm.step.gpios);
	gpio_clear(sm.dir.gpioport, sm.dir.gpios);
}

uint8_t readAS5600Status(){
	uint8_t status = 0;

	i2c_start_addr(I2C1, AS5600_ADDR, Write);
	i2c_write_read(I2C1, AS5600_ADDR, AS5600_STATUS);
	status = i2c_read(I2C1, true);
	i2c_stop(I2C1);

	return status;
}

void readingSensorStatusTask(void* args){
	ReadingTaskData* data = (ReadingTaskData*) args;

	uint32_t i2c = data->i2c;
	uint8_t addr = AS5600_ADDR;
	volatile uint16_t status = 0;

	for(;;){
		if(xSemaphoreTake(*(data->signal), portMAX_DELAY) == pdTRUE){
			gpio_set(IND_PORT, ROTATION_SENSOR_TASK_IND);
			*(data->valueToUpdate) = readAS5600Status();

			vTaskDelay(pdMS_TO_TICKS(100)); // debug pause
			gpio_clear(IND_PORT, ROTATION_SENSOR_TASK_IND);
		} else {
			taskYIELD();
		}
	}
}

void readingRotationTask(void* args) {
	ReadingTaskData* data = (ReadingTaskData*) args;

	uint32_t i2c = data->i2c;
	uint8_t addr = AS5600_ADDR;
	volatile uint16_t angleValue = 0;

	BaseType_t start;
	BaseType_t delay = 100;

	for (;;) {
		if(xSemaphoreTake(*(data->signal), portMAX_DELAY) == pdTRUE){
			gpio_set(IND_PORT, ROTATION_SENSOR_TASK_IND);

			i2c_start_addr(i2c,addr,Write);
			i2c_write_read(i2c, addr, AS5600_ANGLE_L);
			uint8_t byte_1 = i2c_read(i2c, true);
			i2c_stop(i2c);

			i2c_start_addr(i2c,addr,Write);
			i2c_write_read(i2c, addr, AS5600_ANGLE_H);
			uint8_t byte_2 = i2c_read(i2c, true);
			i2c_stop(i2c);

			angleValue = (byte_1 << 8) | byte_2;
			angleValue = angleValue * 360 / 4096;

			*(data->valueToUpdate) = angleValue;

			// blocking debug pause
			start = xTaskGetTickCount();
			while(xTaskGetTickCount() < start + pdMS_TO_TICKS(delay)){
				;
			}

			gpio_clear(IND_PORT, ROTATION_SENSOR_TASK_IND);
		} else {
			taskYIELD();
		}
	}
}

// TODO: Why it doesn't work with uint8_t? 
void decimalToBinary(int decimal, int* binaryArr) {
	int bitCount = 0;
	int internalBuff[8] = {0};
	
	while (decimal > 0) {
		internalBuff[bitCount] = decimal % 2;
		decimal /= 2;
		bitCount++;
	}
	
	for(int i = 7; i >= 0; i--){
		binaryArr[i] = internalBuff[7-i];
	}
}

void stepperMotorTask(void* args){
	StepperMotorTaskData motors = *((StepperMotorTaskData*) args);
	StepperMotor* motorAlpha = motors.motorAlpha;
	StepperMotor* motorBeta = motors.motorBeta;

	BaseType_t xMovementQueueStatus;
	Point receivedPoint;
	for(;;){
		if(xQueueReceive(movementQueue, &receivedPoint, portMAX_DELAY) == pdPASS){
			gpio_set(IND_PORT, STEPPER_MOTOR_TASK_IND);
			uart_puts("New position request: [");
			uart_putn(receivedPoint.alphaDegrees);
			uart_puts(", ");
			uart_putn(receivedPoint.betaDegrees);
			uart_puts("]\n\r");

			uart_puts("Current position: [");
			uart_putn(currentPosition.alphaDegrees);
			uart_puts(", ");
			uart_putn(currentPosition.betaDegrees);
			uart_puts("]\n\r");

			int alphaDiff = getDegreeDiff(currentPosition.alphaDegrees, receivedPoint.alphaDegrees);
			int betaDiff = getDegreeDiff(currentPosition.betaDegrees, receivedPoint.betaDegrees);

			int alphaStepsToMove = int(alphaDiff / (1.8  * GEAR_RATIO * (1.0/MICRO_STEPPING)));
			int betaStepsToMove = int(betaDiff / (1.8  * GEAR_RATIO * (1.0/MICRO_STEPPING)));

			uart_puts("Degrees to move: [");
			uart_putn(alphaDiff);
			uart_puts(", ");
			uart_putn(betaDiff);
			uart_puts("]\n\r");

			uart_puts("Steps to move: [");
			uart_putn(alphaStepsToMove);
			uart_puts(", ");
			uart_putn(betaStepsToMove);
			uart_puts("]\n\r");

			if(alphaStepsToMove > 0){
				gpio_set(motorAlpha->dir.gpioport, motorAlpha->dir.gpios);
			} else {
				gpio_clear(motorAlpha->dir.gpioport, motorAlpha->dir.gpios);
			}

			if(betaStepsToMove > 0){
				gpio_set(motorBeta->dir.gpioport, motorBeta->dir.gpios);
			} else {
				gpio_clear(motorBeta->dir.gpioport, motorBeta->dir.gpios);
			}

			motorAlpha->stepsToMove = abs(alphaStepsToMove);
			motorBeta->stepsToMove = abs(betaStepsToMove);

			while(motorAlpha->stepsToMove > 0 || motorBeta->stepsToMove > 0){
				taskYIELD();
			}

			xSemaphoreGive(readAlphaPositionSignal);
			xSemaphoreGive(readBetaPositionSignal);
			gpio_clear(IND_PORT, STEPPER_MOTOR_TASK_IND);
		} else {
			taskYIELD();
		}
	}
}

enum MainState {
	INITIAL_MOVEMENT,
	BUTTON_WAIT,
	MAIN_MOVEMENT
};


bool generatePoint(int counter, Point& point){
	int n = 6;
	int squareSize = 60; // in degrees for whole movement

	if(counter > n * n) return false;

	int col = counter % n;
	int row = counter / n;
	int x = squareSize / n;

	uart_puts("Point: [");
	uart_putn(col);
	uart_puts(", ");
	uart_putn(row);
	uart_puts("]\n\r");

	if(col == 0 && row == 0){
		point.alphaDegrees += -x * n/2;
		point.betaDegrees += x * n/2;
		return true;
	}

	if(counter == n * n){
		point.alphaDegrees += -x * (n/2-1);
		point.betaDegrees += x * n/2;
		return true;
	}

	if(col == 0){
		point.alphaDegrees += -x * (n-1);
		point.betaDegrees += -x;
		return true;
	}

	point.alphaDegrees += x;

	return true;
}

void mainTask(void* args){
	MainState state = INITIAL_MOVEMENT;
	uint8_t points = 4;

	int change[points][2] = {
		{10, 0},
		{0, 10},
		{-10, 0},
		{0, -10}
	};

	Point point;
	int counter = 0;

	int binaryArr[8] = {0};
	xSemaphoreGive(readAlphaSensorStatusSignal);
	uart_puts("Alpha Status: ");
	decimalToBinary(currentAlphaSensorStatus, binaryArr);
	for(uint8_t i = 0; i < 8; i++){
		uart_putn(binaryArr[i]);
	}
	uart_puts("\n\r");

	xSemaphoreGive(readBetaSensorStatusSignal);
	uart_puts("Beta Status: ");
	decimalToBinary(currentBetaSensorStatus, binaryArr);
	for(uint8_t i = 0; i < 8; i++){
		uart_putn(binaryArr[i]);
	}
	uart_puts("\n\r");

	// TODO: check if the status is correct. Return and light diode otherwise.

	xSemaphoreGive(readAlphaPositionSignal);
	xSemaphoreGive(readBetaPositionSignal);
	uart_puts("Current position: [");
	uart_putn(currentPosition.alphaDegrees);
	uart_puts(", ");
	uart_putn(currentPosition.betaDegrees);
	uart_puts("]\n\r");

	for(;;){
		gpio_set(IND_PORT, MAIN_TASK_IND);
		switch(state){
			case INITIAL_MOVEMENT:
				point.alphaDegrees = currentPosition.alphaDegrees + change[counter][0];
				point.betaDegrees = currentPosition.betaDegrees + change[counter][1];
				xQueueSendToBack(movementQueue, &point, portMAX_DELAY);

				counter++;
				if(counter>=points){
					counter = 0;
					state = BUTTON_WAIT;
				}
			break;
			case BUTTON_WAIT:
				gpio_set(GPIOA, STEPPER_MOTOR_SLEEP);
				while(gpio_get(GPIOA, INPUT_BUTTON) == 0){
					;
				}
				gpio_clear(GPIOA, STEPPER_MOTOR_SLEEP);
				state = MAIN_MOVEMENT;
			break;
			case MAIN_MOVEMENT:
				point.alphaDegrees = currentPosition.alphaDegrees;
				point.betaDegrees = currentPosition.betaDegrees;
				if(generatePoint(counter++, point)){
					xQueueSendToBack(movementQueue, &point, portMAX_DELAY);
				} else {
					counter = 0;
					state = BUTTON_WAIT;
				}
			break;
		}

		gpio_clear(IND_PORT, MAIN_TASK_IND);
	}
}

int main(void) {
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);	// CPU clock is 72 MHz

	/* Setup GPIO */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	gpio_set_mode(
		GPIOC,
		GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		DIODE_GPIO);

	gpio_set_mode(
		GPIOA,
		GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		MAIN_TASK_IND|STEPPER_MOTOR_TASK_IND|ROTATION_SENSOR_TASK_IND|STEPPER_MOTOR_SLEEP);

	gpio_set_mode(
		GPIOA,
		GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_PULL_UPDOWN,
		INPUT_BUTTON);

	/* Setup motor GPIO */

	motorAlpha.step.gpioport = GPIOB;
	motorAlpha.step.gpios = GPIO12;

	motorAlpha.dir.gpioport = GPIOB;
	motorAlpha.dir.gpios = GPIO13;

	stepperMotorInit(motorAlpha);

	motorBeta.step.gpioport = GPIOB;
	motorBeta.step.gpios = GPIO14;

	motorBeta.dir.gpioport = GPIOA;
	motorBeta.dir.gpios = GPIO8;

	stepperMotorInit(motorBeta);

	/* Setup UART */
	uart_setup();

	/* Setup I2C */
	rcc_periph_clock_enable(RCC_AFIO);

	rcc_periph_clock_enable(RCC_I2C1);
	gpio_set_mode(GPIOB,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
		GPIO_I2C1_SCL | GPIO_I2C1_SDA);
		
	i2c_setup(I2C1);

	rcc_periph_clock_enable(RCC_I2C2);
	gpio_set_mode(GPIOB,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
		GPIO_I2C2_SCL | GPIO_I2C2_SDA);
		
	i2c_setup(I2C2);

	/* Setup timer */
	rcc_enable_rtc_clock();
	rtc_awake_from_off(RCC_HSE); 
	rtc_set_prescale_val(100); // 625
	nvic_enable_irq(NVIC_RTC_IRQ);
	rtc_clear_flag(RTC_SEC);
	rtc_interrupt_enable(RTC_SEC);

	/* Setup other */
	movementQueue = xQueueCreate(1, sizeof(Point)); // Mailbox-type queue with one element
	currentPosition = {
		.alphaDegrees = ALPHA_OFFSET, 
		.betaDegrees = BETA_OFFSET
	};

	readAlphaPositionSignal = xSemaphoreCreateBinary();
	readBetaPositionSignal = xSemaphoreCreateBinary();
	readAlphaSensorStatusSignal = xSemaphoreCreateBinary();
	readBetaSensorStatusSignal = xSemaphoreCreateBinary();

	alphaReadingTaskData = {
		.i2c = I2C1,
		.signal = &readAlphaPositionSignal,
		.valueToUpdate = &(currentPosition.alphaDegrees)
	};

	betaReadingTaskData = {
		.i2c = I2C2,
		.signal = &readBetaPositionSignal,
		.valueToUpdate = &(currentPosition.betaDegrees)
	};

	alphaReadingStatusTaskData = {
		.i2c = I2C1,
		.signal = &readAlphaSensorStatusSignal,
		.valueToUpdate = &(currentAlphaSensorStatus)
	};

	betaReadingStatusTaskData = {
		.i2c = I2C2,
		.signal = &readBetaSensorStatusSignal,
		.valueToUpdate = &(currentBetaSensorStatus)
	};

	stepperMotorTaskData = {
		.motorAlpha = &motorAlpha,
		.motorBeta = &motorBeta
	};

	uart_puts("Start...\n\r");
	xTaskCreate(mainTask, "MAIN", 100, NULL, 1, NULL);
	xTaskCreate(uartTask, "UART", 100, NULL, 2, NULL);
	xTaskCreate(stepperMotorTask, "SM1", 100, (void*) &stepperMotorTaskData, 3, NULL);

	// reading position and status needs to have higher priority than stepper motor to pre-emtive it
	xTaskCreate(readingRotationTask, "READING_ALPHA_POS", 100, (void*) &alphaReadingTaskData, 4, NULL); 
	xTaskCreate(readingRotationTask, "READING_BETA_POS", 100, (void*) &betaReadingTaskData, 4, NULL);
	xTaskCreate(readingSensorStatusTask, "READING_ALPHA_STATUS", 100, (void*) &alphaReadingStatusTaskData, 4, NULL);
	xTaskCreate(readingSensorStatusTask, "READING_BETA_STATUS", 100, (void*) &betaReadingStatusTaskData, 4, NULL);

	vTaskStartScheduler();
	for (;;);
	return 0;
}

void rtc_isr(void) {
	if ( rtc_check_flag(RTC_SEC) ) {
		rtc_clear_flag(RTC_SEC);

		if(motorAlpha.stepsToMove > 0){
			gpio_set(motorAlpha.step.gpioport, motorAlpha.step.gpios);
			gpio_clear(motorAlpha.step.gpioport, motorAlpha.step.gpios);

			motorAlpha.stepsToMove--;
		}

		if(motorBeta.stepsToMove > 0){
			gpio_set(motorBeta.step.gpioport, motorBeta.step.gpios);
			gpio_clear(motorBeta.step.gpioport, motorBeta.step.gpios);

			motorBeta.stepsToMove--;
		}
	}
}

/* End */

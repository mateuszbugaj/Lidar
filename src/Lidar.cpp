#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

extern "C"{
	#include "uart_lib.h"
	#include "i2c.h"
}

#define COIL_NUMBER 4
#define AS5600_ADDR 0x36
#define AS5600_STATUS 0x0B
#define AS5600_ANGLE_L 0x0C
#define AS5600_ANGLE_H 0x0D

static uint32_t i2c1 = I2C1;
static uint32_t i2c2 = I2C2;

struct Pin {
	uint32_t gpioport;
	uint16_t gpios;
};

struct StepperMotor {
	Pin step;
	Pin dir;
};

static StepperMotor motor1;

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

	// Show bit 3, 4 and 5 of AS5600_STATUS
	uart_puts("Status: ");
	uart_putn((status & (1 << 5) >> 5)); 
	uart_puts(" | ");
	uart_putn((status & (1 << 4) >> 4));
	uart_puts(" | ");
	uart_putn((status & (1 << 3) >> 3));
	uart_puts(" | ");

	return status;
}

void readingRotationTask(void* args) {
	uint32_t i2c = *((uint32_t*) args);

	uint16_t counter = 0;
	uint8_t addr = AS5600_ADDR;
	volatile uint16_t angleValue = 0;

	for (;;) {
		if(i2c == i2c1)
			uart_puts("I2C1");
		else if(i2c == i2c2)
			uart_puts("I2C2");
		else
			uart_puts("I2C?");
		
		uart_puts(": ");

		// (void) readAS5600Status();

		// read rotation angle
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

		uart_putn(angleValue);
		uart_puts("\n\r");

		gpio_toggle(GPIOC,GPIO13);
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

void stepperMotorTask(void* args){
	StepperMotor* motor = (StepperMotor*) args;

	uint16_t steps = 1000;
	for(;;){
		for(int step = 0; step < steps; step++){
			gpio_set(motor->step.gpioport, motor->step.gpios);
			vTaskDelay(pdMS_TO_TICKS(5));
			gpio_clear(motor->step.gpioport, motor->step.gpios);
			vTaskDelay(pdMS_TO_TICKS(5));
		}

		gpio_toggle(motor->dir.gpioport, motor->dir.gpios);
	}
}

int main(void) {
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);	// CPU clock is 72 MHz

	/* Setup diode GPIO */
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(
		GPIOC,
		GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		GPIO13);

	/* Setup motor GPIO */
	rcc_periph_clock_enable(RCC_GPIOB);

	motor1.step.gpioport = GPIOB;
	motor1.step.gpios = GPIO12;

	motor1.dir.gpioport = GPIOB;
	motor1.dir.gpios = GPIO13;

	stepperMotorInit(motor1);

	/* Setup UART */
	uart_setup();

	/* Setup I2C */
	rcc_periph_clock_enable(RCC_AFIO);

	rcc_periph_clock_enable(RCC_I2C1);
	gpio_set_mode(GPIOB,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
		GPIO_I2C1_SCL | GPIO_I2C1_SDA);
		
	i2c_setup(i2c1);

	rcc_periph_clock_enable(RCC_I2C2);
	gpio_set_mode(GPIOB,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
		GPIO_I2C2_SCL | GPIO_I2C2_SDA);
		
	i2c_setup(i2c2);

	uart_puts("Start...\n\r");
	xTaskCreate(uartTask,"UART",100,NULL,configMAX_PRIORITIES-1,NULL);
	xTaskCreate(readingRotationTask,"READING_ALPHA",800, (void*) &i2c1,configMAX_PRIORITIES-1,NULL);
	xTaskCreate(readingRotationTask,"READING_BETA",800, (void*) &i2c2,configMAX_PRIORITIES-1,NULL);
	xTaskCreate(stepperMotorTask, "SM1", 100, (void*) &motor1, configMAX_PRIORITIES-1, NULL);

	vTaskStartScheduler();
	for (;;);
	return 0;
}

/* End */

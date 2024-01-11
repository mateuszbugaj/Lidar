#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

extern "C"{
	#include "uart_lib.h"
}

#define COIL_NUMBER 4

struct Pin {
	uint32_t gpioport;
	uint16_t gpios;
};

struct StepperMotor {
	Pin coils[COIL_NUMBER];
	const uint8_t phases[8] {1, 3, 2, 6, 4, 12, 8, 9};
	int8_t lastPhase;
};

void stepperMotorInit(StepperMotor& sm){
	for(uint8_t i = 0; i < COIL_NUMBER; i++){
		gpio_set_mode(
			sm.coils[i].gpioport,
			GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL,
			sm.coils[i].gpios
		);

		gpio_clear(sm.coils[i].gpioport, sm.coils[i].gpios);
	}
}

void demoTask(void *args __attribute__((unused))) {
	uint16_t counter = 0;
	for (;;) {
		uart_putn(counter++);
		uart_puts("\n\r");

		gpio_toggle(GPIOC,GPIO13);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void decimalToBinary(int value, int digits, uint8_t *arr) {
	for (int i = 0; value > 0; i++) {
		arr[i] = value % 2;
		value /= 2;
		digits--;
	}
}

void stepperMotorTask(void* args){
	StepperMotor* motor = (StepperMotor*) args;

	uint8_t dir = 1;
	uint16_t steps = 1000;
	for(;;){
		for(int step = 0; step < steps; step++){
			uint8_t c[COIL_NUMBER] = {0};
			motor->lastPhase += dir;

			if(motor->lastPhase == 8) motor->lastPhase = 0;
			if(motor->lastPhase == -1) motor->lastPhase = 7;

			decimalToBinary(motor->phases[motor->lastPhase], COIL_NUMBER, c);

			for(uint8_t coilNumber = 0; coilNumber < COIL_NUMBER; coilNumber++){
				if(c[coilNumber]){
					gpio_set(motor->coils[coilNumber].gpioport, motor->coils[coilNumber].gpios);
				} else {
					gpio_clear(motor->coils[coilNumber].gpioport, motor->coils[coilNumber].gpios); // ERROR
				}
			}

			vTaskDelay(pdMS_TO_TICKS(10));
		}

		dir *= -1;
	}
}

StepperMotor motor1, motor2;

int main(void) {
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);	// CPU clock is 72 MHz

	// GPIO PC13:
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(
		GPIOC,
		GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		GPIO13);

	rcc_periph_clock_enable(RCC_GPIOB);

	motor1.coils[0].gpioport = GPIOB;
	motor1.coils[0].gpios = GPIO12;

	motor1.coils[1].gpioport = GPIOB;
	motor1.coils[1].gpios = GPIO13;

	motor1.coils[2].gpioport = GPIOB;
	motor1.coils[2].gpios = GPIO14;

	motor1.coils[3].gpioport = GPIOB;
	motor1.coils[3].gpios = GPIO15;

	stepperMotorInit(motor1);

	motor2.coils[0].gpioport = GPIOB;
	motor2.coils[0].gpios = GPIO6;

	motor2.coils[1].gpioport = GPIOB;
	motor2.coils[1].gpios = GPIO7;

	motor2.coils[2].gpioport = GPIOB;
	motor2.coils[2].gpios = GPIO8;

	motor2.coils[3].gpioport = GPIOB;
	motor2.coils[3].gpios = GPIO9;

	stepperMotorInit(motor2);

	uart_setup();

	
	xTaskCreate(uartTask,"UART",100,NULL,configMAX_PRIORITIES-1,NULL);
	xTaskCreate(demoTask,"DEMO",100,NULL,configMAX_PRIORITIES-1,NULL);

	uart_puts("Start...\n\r");

	BaseType_t sm1Result = xTaskCreate(stepperMotorTask, "SM1", 100, (void*) &motor1, configMAX_PRIORITIES-1, NULL);
	// BaseType_t sm2Result = xTaskCreate(stepperMotorTask, "SM2", 100, (void*) &motor2, configMAX_PRIORITIES-1, NULL);

	vTaskStartScheduler();
	for (;;);
	return 0;
}

/* End */

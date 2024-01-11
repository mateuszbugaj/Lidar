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
	Pin step;
	Pin dir;
};

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

void demoTask(void *args __attribute__((unused))) {
	uint16_t counter = 0;
	for (;;) {
		uart_putn(counter++);
		uart_puts("\n\r");

		gpio_toggle(GPIOC,GPIO13);
		vTaskDelay(pdMS_TO_TICKS(1000));
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

StepperMotor motor1;

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

	motor1.step.gpioport = GPIOB;
	motor1.step.gpios = GPIO12;

	motor1.dir.gpioport = GPIOB;
	motor1.dir.gpios = GPIO13;

	stepperMotorInit(motor1);

	uart_setup();

	xTaskCreate(uartTask,"UART",100,NULL,configMAX_PRIORITIES-1,NULL);
	xTaskCreate(demoTask,"DEMO",100,NULL,configMAX_PRIORITIES-1,NULL);

	uart_puts("Start...\n\r");

	BaseType_t sm1Result = xTaskCreate(stepperMotorTask, "SM1", 100, (void*) &motor1, configMAX_PRIORITIES-1, NULL);

	vTaskStartScheduler();

	for (;;);
	return 0;
}

/* End */

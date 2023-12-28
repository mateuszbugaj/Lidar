#include "FreeRTOS.h"
#include "task.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

extern void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
	for(;;);	// Loop forever here..
}

static void
task1(void *args __attribute((unused))) {

	for (;;) {
		gpio_toggle(GPIOC,GPIO13);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

int
main(void) {

	rcc_clock_setup_in_hse_8mhz_out_72mhz(); // For "blue pill"

	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(
		GPIOC,
		GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		GPIO13);

	xTaskCreate(task1,"LED",1000,NULL,configMAX_PRIORITIES-1,NULL);
	vTaskStartScheduler();

	for (;;);
	return 0;
}

// End

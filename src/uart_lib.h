#pragma once
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

QueueHandle_t uart_txq;		// TX queue for UART

static void uart_setup(void) {
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);

	// UART TX on PA9 (GPIO_USART1_TX)
	gpio_set_mode(GPIOA,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		GPIO_USART1_TX);

	usart_set_baudrate(USART1,38400);
	usart_set_databits(USART1,8);
	usart_set_stopbits(USART1,USART_STOPBITS_1);
	usart_set_mode(USART1,USART_MODE_TX);
	usart_set_parity(USART1,USART_PARITY_NONE);
	usart_set_flow_control(USART1,USART_FLOWCONTROL_NONE);
	usart_enable(USART1);

	// Create a queue for data to transmit from UART
	uart_txq = xQueueCreate(256,sizeof(char));
}

static void uartTask(void *args __attribute__((unused))) {
	char ch;

	for (;;) {
		// Receive char to be TX
		if ( xQueueReceive(uart_txq,&ch,500) == pdPASS ) {
			while ( !usart_get_flag(USART1,USART_SR_TXE) )
				taskYIELD();	// Yield until ready
			usart_send(USART1,ch);
		}
	}
}

static void uart_puts(const char *s) {
	for ( ; *s; ++s ) {
		xQueueSend(uart_txq,s,portMAX_DELAY); 
	}
}

static void uart_putn(uint32_t n) {
	char buff[10] = {'0'};
	uint8_t digitNumber = 0;

	if(n == 0) digitNumber = 1;

	while(n > 0){
		char c = n % 10 + '0';
		buff[digitNumber++] = c;

		n /= 10;
	}

	while(digitNumber > 0){
		xQueueSend(uart_txq,&buff[digitNumber-1],portMAX_DELAY);
		digitNumber--;
	}
}
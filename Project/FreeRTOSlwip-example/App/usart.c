/*
 * usart.c
 *
 *  Created on: Apr 14, 2014
 *      Author: onikitenko
 */

#include "usart.h"
#include "stm32f2xx_hal.h"
#include "stm32f2xx_hal_uart.h"
#include "FreeRTOS.h"
#include "../../FreeRTOS/inc/task.h"

extern UART_HandleTypeDef UartHandle;

void usart_putchar(char t)
{
	taskENTER_CRITICAL();
	if (t == '\n')				// To avoid non-correct logs
		usart_putchar('\r');

	char size_to_transmit;

	size_to_transmit = sizeof(t);

	while (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_TXE) == RESET);
		HAL_UART_Transmit(&UartHandle, &t, size_to_transmit, 2000); // Timeout was 100, size to trans. was 1
		taskEXIT_CRITICAL();
};

void usart_putstr(char *str)
{
	taskENTER_CRITICAL();

	do {
		usart_putchar(*str++);
	} while(*str!='\0');

	taskEXIT_CRITICAL();
}

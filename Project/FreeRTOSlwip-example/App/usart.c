/*
 * usart.c
 *
 *  Created on: Apr 14, 2014
 *      Author: onikitenko
 */

#include "usart.h"
#include "stm32f2xx_hal.h"
#include "stm32f2xx_hal_uart.h"

extern UART_HandleTypeDef UartHandle;

void usart_putchar(char t)
{
	while (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_TXE) == RESET);
		HAL_UART_Transmit(&UartHandle, &t, 1, 100);
};

void usart_putstr(char *str)
{
	do {
		usart_putchar(*str++);
	} while(*str!='\0');
}

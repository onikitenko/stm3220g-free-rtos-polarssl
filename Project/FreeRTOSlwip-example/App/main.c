/*
 * main.c
 *
 *  Created on: Apr 9, 2014
 *      Author: onikitenko
 */

//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
//#include "stm32f2xx.h"
//#include "stm32f2xx_hal.h"
//#include "stm32f2xx_hal_gpio.h"
//#include "stm32f2xx_hal_rcc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "stm32f2xx_hal.h"
#include "stm32f2xx_hal_uart.h"

static GPIO_InitTypeDef  GPIO_InitStruct;
UART_HandleTypeDef UartHandle;
static void SystemClock_Config(void);
static void Error_Handler(void);

void usart_putchar(char t);
void usart_putstr(char *str);

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

void vFreeRTOSInitAll()
{

	  /*##-1- Enable peripherals and GPIO Clocks #################################*/
	  /* Enable GPIO TX/RX clock */
	  USARTx_TX_GPIO_CLK_ENABLE();
	  USARTx_RX_GPIO_CLK_ENABLE();

	  /* Enable USARTx clock */
	  USARTx_CLK_ENABLE();

	  /*##-2- Configure peripheral GPIO ##########################################*/
	  /* UART TX GPIO pin configuration  */
	  GPIO_InitStruct.Pin       = USARTx_TX_PIN;
	  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull      = GPIO_PULLUP;
	  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	  GPIO_InitStruct.Alternate = USARTx_TX_AF;

	  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

	  /* UART RX GPIO pin configuration  */
	  GPIO_InitStruct.Pin = USARTx_RX_PIN;
	  GPIO_InitStruct.Alternate = USARTx_RX_AF;

	  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

	  /*##-1- Configure the UART peripheral ######################################*/
	  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	  /* UART1 configured as follow:
	      - Word Length = 8 Bits
	      - Stop Bit    = One Stop bit
	      - Parity      = ODD parity
	      - BaudRate    = 9600 baud
	      - Hardware flow control disabled (RTS and CTS signals) */
	  UartHandle.Instance        = USARTx;

	  UartHandle.Init.BaudRate   = 115200;
	  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	  UartHandle.Init.StopBits   = UART_STOPBITS_1;
	  UartHandle.Init.Parity     = UART_PARITY_NONE;
	  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	  UartHandle.Init.Mode       = UART_MODE_TX_RX;

	  if (HAL_UART_Init(&UartHandle) != HAL_OK)
	  {
	    /* Initialization Error */
	    Error_Handler();
	  }

}

void vLedTask (void *pvParameters)
{
    while(1)
    {
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_6);
        /* Insert delay 100 ms */
        vTaskDelay(100);
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_8);
        /* Insert delay 100 ms */
        vTaskDelay(100);
        HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_9);
        /* Insert delay 100 ms */
        vTaskDelay(100);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
        /* Insert delay 100 ms */
        vTaskDelay(100);

        usart_putstr("ololo");
        usart_putchar('t');
    }
    vTaskDelete(NULL);
}


int main()
{
    vFreeRTOSInitAll();
    xTaskCreate(vLedTask,(signed char*)"LedTask", configMINIMAL_STACK_SIZE,
					NULL, tskIDLE_PRIORITY + 1, NULL);
    vTaskStartScheduler();
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 120000000
  *            HCLK(Hz)                       = 120000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 240
  *            PLL_P                          = 2
  *            PLL_Q                          = 5
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
}

static void Error_Handler(void)
{
  /* Turn LED3 on */
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
  while (1)
  {
  }
}

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

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
#include "stm32f2xx_hal.h"

static GPIO_InitTypeDef  GPIO_InitStruct;
static void SystemClock_Config(void);

void vFreeRTOSInitAll()
{
	 /* This sample code shows how to use STM32F2xx GPIO HAL API to toggle PG6, PG7,
	    PG10, and PG12 IOs (connected to LED1, LED2, LED3 and LED4 on STM322xG-EVAL board)
	    in an infinite loop.
	    To proceed, 3 steps are required: */

	  /* STM32F2xx HAL library initialization:
	       - Configure the Flash prefetch, instruction and Data caches
	       - Configure the Systick to generate an interrupt each 1 msec
	       - Set NVIC Group Priority to 4
	       - Global MSP (MCU Support Package) initialization
	     */
	  HAL_Init();
	  /* Configure the system clock */
	  SystemClock_Config();

	  /* -1- Enable GPIOG, GPIOC and GPIOI Clock (to be able to program the configuration registers) */
	  __GPIOG_CLK_ENABLE();
	  __GPIOC_CLK_ENABLE();
	  __GPIOI_CLK_ENABLE();

	  /* -2- Configure PG.6, PG.8, PI.9 and PC.7 IOs in output push-pull mode to
	         drive external LEDs */
	  GPIO_InitStruct.Pin = (GPIO_PIN_6 | GPIO_PIN_8);
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

	  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = GPIO_PIN_9;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

	  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = GPIO_PIN_7;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
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

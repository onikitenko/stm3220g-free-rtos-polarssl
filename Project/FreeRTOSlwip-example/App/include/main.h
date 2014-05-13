/**
  ******************************************************************************
  * @file    LwIP/LwIP_TCP_Echo_Client/Inc/main.h 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    13-March-2014
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_hal.h"
#include "stm322xg_eval.h"
#include "stdio.h"

#define USARTx                           USART3
#define USARTx_CLK_ENABLE()              __USART3_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __USART3_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __USART3_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_10
#define USARTx_TX_GPIO_PORT              GPIOC
#define USARTx_TX_AF                     GPIO_AF7_USART3
#define USARTx_RX_PIN                    GPIO_PIN_11
#define USARTx_RX_GPIO_PORT              GPIOC
#define USARTx_RX_AF                     GPIO_AF7_USART3

#define LEDn                             4

#define LED1_PIN                         GPIO_PIN_6
#define LED1_GPIO_PORT                   GPIOG
#define LED1_GPIO_CLK_ENABLE()           __GPIOG_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()          __GPIOG_CLK_DISABLE()

#define LED2_PIN                         GPIO_PIN_8
#define LED2_GPIO_PORT                   GPIOG
#define LED2_GPIO_CLK_ENABLE()           __GPIOG_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()          __GPIOG_CLK_DISABLE()

#define LED3_PIN                         GPIO_PIN_9
#define LED3_GPIO_PORT                   GPIOI
#define LED3_GPIO_CLK_ENABLE()           __GPIOI_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()          __GPIOI_CLK_DISABLE()

#define LED4_PIN                         GPIO_PIN_7
#define LED4_GPIO_PORT                   GPIOC
#define LED4_GPIO_CLK                    RCC_AHB1Periph_GPIOC
#define LED4_GPIO_CLK_ENABLE()           __GPIOC_CLK_ENABLE()
#define LED4_GPIO_CLK_DISABLE()          __GPIOC_CLK_DISABLE()
#define LEDx_GPIO_CLK_ENABLE(__INDEX__)   (((__INDEX__) == 0) ? LED1_GPIO_CLK_ENABLE() :\
                                           ((__INDEX__) == 1) ? LED2_GPIO_CLK_ENABLE() :\
                                           ((__INDEX__) == 2) ? LED3_GPIO_CLK_ENABLE() : LED4_GPIO_CLK_ENABLE())

#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  (((__INDEX__) == 0) ? LED1_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 1) ? LED2_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 2) ? LED3_GPIO_CLK_DISABLE() : LED4_GPIO_CLK_DISABLE())

 /** @addtogroup STM322xG_EVAL_LOW_LEVEL_BUTTON
   * @{
   */
 /* Joystick pins are connected to IO Expander (accessible through I2C1 interface) */
 #define BUTTONn                            3


 /**
   * @brief Wakeup push-button
   */
 #define WAKEUP_BUTTON_PIN                   GPIO_PIN_0
 #define WAKEUP_BUTTON_GPIO_PORT             GPIOA
 #define WAKEUP_BUTTON_GPIO_CLK_ENABLE()     __GPIOA_CLK_ENABLE()
 #define WAKEUP_BUTTON_GPIO_CLK_DISABLE()    __GPIOA_CLK_DISABLE()
 #define WAKEUP_BUTTON_EXTI_IRQn             EXTI0_IRQn

 /**
   * @brief Tamper push-button
   */
 #define TAMPER_BUTTON_PIN                    GPIO_PIN_13
 #define TAMPER_BUTTON_GPIO_PORT              GPIOC
 #define TAMPER_BUTTON_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()
 #define TAMPER_BUTTON_GPIO_CLK_DISABLE()     __GPIOC_CLK_DISABLE()
 #define TAMPER_BUTTON_EXTI_IRQn              EXTI15_10_IRQn

 /**
   * @brief Key push-button
   */
 #define KEY_BUTTON_PIN                       GPIO_PIN_15
 #define KEY_BUTTON_GPIO_PORT                 GPIOG
 #define KEY_BUTTON_GPIO_CLK_ENABLE()         __GPIOG_CLK_ENABLE()
 #define KEY_BUTTON_GPIO_CLK_DISABLE()        __GPIOG_CLK_DISABLE()
 #define KEY_BUTTON_EXTI_IRQn                 EXTI15_10_IRQn

 #define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    (((__INDEX__) == 0) ? WAKEUP_BUTTON_GPIO_CLK_ENABLE() :\
                                                ((__INDEX__) == 1) ? TAMPER_BUTTON_GPIO_CLK_ENABLE() : KEY_BUTTON_GPIO_CLK_ENABLE())

 #define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)    (((__INDEX__) == 0) ? WAKEUP_BUTTON_GPIO_CLK_DISABLE() :\
                                                 ((__INDEX__) == 1) ? TAMPER_BUTTON_GPIO_CLK_DISABLE() : KEY_BUTTON_GPIO_CLK_DISABLE())

 /* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define DEST_IP_ADDR0   192
#define DEST_IP_ADDR1   168
#define DEST_IP_ADDR2   1
#define DEST_IP_ADDR3   1

#define DEST_PORT       4433
 
/*Static IP ADDRESS: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3 */
#define IP_ADDR0   192
#define IP_ADDR1   168
#define IP_ADDR2   1
#define IP_ADDR3   10
   
/*NETMASK*/
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   255
#define NETMASK_ADDR3   0

/*Gateway Address*/
#define GW_ADDR0   192
#define GW_ADDR1   168
#define GW_ADDR2   0
#define GW_ADDR3   1  

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

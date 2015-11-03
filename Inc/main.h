/**
  ******************************************************************************
  * @file    Cam_Soft/Inc/main.h 
  * @author  Abhas Maskey
  * @version V1.0
  * @date    27-October-2015
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 SNUSAT-I</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f429i_discovery.h"
#include "stm32fxxx_hal.h"

/* TM Library Includes -------------------------------------------------------*/
#include "defines.h"
#include "tm_stm32_usart.h"
#include "tm_stm32_gpio.h"
#include "tm_stm32_delay.h"
#include "tm_stm32_buffer.h"
#include "tm_stm32_i2c.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Definition for USARTx clock resources */
#define USARTx                          USART1
#define USARTx_CLK_ENABLE()             __HAL_RCC_USART1_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()            __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()          __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                   GPIO_PIN_9
#define USARTx_TX_GPIO_PORT             GPIOA
#define USARTx_TX_AF                    GPIO_AF7_USART1
#define USARTx_RX_PIN                   GPIO_PIN_10
#define USARTx_RX_GPIO_PORT             GPIOA
#define USARTx_RX_AF                    GPIO_AF7_USART1

/* Timeout for USART */
#define USART_TIMEOUT                   5000
/* Size of Transmission buffer */
#define TXBUFFERSIZE                    (COUNTOF(aTxBuffer)-1)

/*Size of Reception buffer */
#define RXBUFFERSIZE                    TXBUFFERSIZE

/* Definition for I2Cx Pins */
#define I2C_TYPE                        I2C1
#define POA030R_ADD_BIT_SHIFT           0xDC /* 7bit<<1, same as write address*/
#define POA030R_WRITE_ADD               0xDC 
#define POA030R_READ_ADD                0xDD
#define POA030R_REG_GR_ADD              0x03
#define POA030R_REG_GROUPA              0x00
#define POA030R_REG_GROUPB              0x01
#define POA030R_REG_GROUPC              0x02
/* Exported macro ------------------------------------------------------------*/

#define COUNTOF(__BUFFER__)         (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

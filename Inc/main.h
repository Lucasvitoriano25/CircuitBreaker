/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define IBAT_Pin GPIO_PIN_0
#define IBAT_GPIO_Port GPIOA
#define VBAT_Pin GPIO_PIN_1
#define VBAT_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_4
#define SW1_GPIO_Port GPIOA
#define SW1_EXTI_IRQn EXTI4_15_IRQn
#define SW3_Pin GPIO_PIN_5
#define SW3_GPIO_Port GPIOA
#define SW4_Pin GPIO_PIN_6
#define SW4_GPIO_Port GPIOA
#define SW5_Pin GPIO_PIN_7
#define SW5_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_0
#define SW2_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOB
#define CMD_MOSFET_Pin GPIO_PIN_8
#define CMD_MOSFET_GPIO_Port GPIOA
#define LCD_D7_Pin GPIO_PIN_9
#define LCD_D7_GPIO_Port GPIOA
#define LCD_D6_Pin GPIO_PIN_10
#define LCD_D6_GPIO_Port GPIOA
#define LCD_D5_Pin GPIO_PIN_11
#define LCD_D5_GPIO_Port GPIOA
#define LCD_D4_Pin GPIO_PIN_12
#define LCD_D4_GPIO_Port GPIOA
#define LCD_E_Pin GPIO_PIN_15
#define LCD_E_GPIO_Port GPIOA
#define LCD_RW_Pin GPIO_PIN_3
#define LCD_RW_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_4
#define LCD_RS_GPIO_Port GPIOB
#define CMD_BUZZER_Pin GPIO_PIN_5
#define CMD_BUZZER_GPIO_Port GPIOB
#define TX_Pin GPIO_PIN_6
#define TX_GPIO_Port GPIOB
#define RX_Pin GPIO_PIN_7
#define RX_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define DEBUG_ON() HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)
#define DEBUG_OFF() HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)
#define DEBUG_TOGGLE() HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)

#define MOSFET_ON() HAL_GPIO_WritePin(CMD_MOSFET_GPIO_Port, CMD_MOSFET_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)
#define MOSFET_OFF() HAL_GPIO_WritePin(CMD_MOSFET_GPIO_Port, CMD_MOSFET_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)
#define MOSFET_TOGGLE() HAL_GPIO_TogglePin(CMD_MOSFET_GPIO_Port, CMD_MOSFET_Pin);HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)
    
#define FLASH_USER_ADDR 0x08007000
#define ganhoINA        0.048051

#define OC_BUFF_SIZE 1138

// Circuit-Break states
typedef enum {
  IDLE = 0,
  NORMAL,
  BREAK
} E_state;

// IBreak edition states
typedef enum {
  HUNDREDS = 0,
  TENS,
  UNITS,
  TENTHS = 4,
  HUNDREDTHS
} D_state;

// Buzzer states
typedef enum {
  MUTE = 0,
  SOUND
} B_state;

// Circuit Break struct
typedef struct {
  E_state  state;
  D_state  digit;
  B_state  buzzer;
  uint16_t ibat;
  uint16_t vbat;
  uint16_t ibreak;
  float    ibat_float;
  float    vbat_float;
  float    ibreak_float;
} CircuitBreak;
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

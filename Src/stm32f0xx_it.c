/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"

/* USER CODE BEGIN 0 */
#include "lcd.h"
#include "main.h"

extern float last_ibats[3];
extern float last_ibats_filtered[3];
extern float last_vbats[3];
extern float last_vbats_filtered[3];

extern uint8_t uart_rx_data[2];

extern uint16_t oc_buff[OC_BUFF_SIZE];
extern uint16_t oc_idx;
extern uint16_t oc_buff_head;
extern uint8_t  oc_go;

uint16_t sample_idx = 0;
uint32_t ibat_accum = 0;
uint32_t ibat_final = 0;
uint32_t vbat_accum = 0;
uint32_t vbat_final = 0;

extern CircuitBreak cbs;
extern uint32_t last_psh_tick;
extern uint32_t beepBreakStart;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim16;

/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    HAL_Delay(100);
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line 4 to 15 interrupts.
*/
void EXTI4_15_IRQHandler(void) {
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
  if (HAL_GetTick() - last_psh_tick > 200) {
    HAL_Delay(10);
    if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET) {
      MOSFET_TOGGLE();
      oc_go = 1;
      cbs.state = NORMAL;
    }
    
    last_psh_tick = HAL_GetTick();
  }
  /* USER CODE END EXTI4_15_IRQn 0 */
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel 1 interrupt.
*/
void DMA1_Channel1_IRQHandler(void) {
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
  // Clear the transfer complete flag
  hdma_adc.DmaBaseAddress->IFCR = DMA_FLAG_TC1 << hdma_adc.ChannelIndex;
  
  
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  
  // Check if break condition and turn the MOSFET off
  if (cbs.ibat > (cbs.ibreak+2)) {
    //MOSFET_OFF();
    
    // Turn BUZZER on
    if (cbs.state != BREAK && cbs.buzzer == SOUND) {
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
      beepBreakStart = HAL_GetTick();
    }
    
    cbs.state = BREAK;
    //oc_go = 0;
   
  }
  
  /* USER CODE END DMA1_Channel1_IRQn 0 */
    
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */
  ibat_accum += cbs.ibat;
  vbat_accum += cbs.vbat;
  sample_idx++;
  if (sample_idx == 1999) {
    sample_idx = 0;
    ibat_final = ibat_accum;
    vbat_final = vbat_accum;
    ibat_accum = 0;
    vbat_accum = 0;
  }
  
  // Fill over current buffer
  if (oc_go && oc_idx != OC_BUFF_SIZE) {
    
    oc_buff[oc_idx] = cbs.ibat;
    oc_idx++;
    
} 
  
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */
  __HAL_TIM_CLEAR_IT(&htim16, TIM_IT_UPDATE);
  
  // Update last_ibats
  last_ibats[2] = last_ibats[1];
  last_ibats[1] = last_ibats[0];
  last_ibats[0] = (ibat_final * ganhoINA) / 1999;

  // Update previous last_ibats_filtered
  last_ibats_filtered[2] = last_ibats_filtered[1];
  last_ibats_filtered[1] = last_ibats_filtered[0];
  
  // Calculate ibat filtered
  last_ibats_filtered[0] = 0.494029456382636*last_ibats[0] + 0.988058912765273*last_ibats[1] + 0.494029456382636*last_ibats[2] - 0.709971972349332*last_ibats_filtered[1] - 0.266145853181214*last_ibats_filtered[2];
  
  // Pass to the struct cbs
  cbs.ibat_float = last_ibats_filtered[0] > 0 ? last_ibats_filtered[0] : 0;
  
  // Update last_vbats
  last_vbats[2] = last_vbats[1];
  last_vbats[1] = last_vbats[0];
  last_vbats[0] = (vbat_final/1999)*3.3*11/4095;

  // Update previous last_ibats_filtered
  last_vbats_filtered[2] = last_vbats_filtered[1];
  last_vbats_filtered[1] = last_vbats_filtered[0];
  
  // Calculate ibat filtered
  last_vbats_filtered[0] = 0.494029456382636*last_vbats[0] + 0.988058912765273*last_vbats[1] + 0.494029456382636*last_vbats[2] - 0.709971972349332*last_vbats_filtered[1] - 0.266145853181214*last_vbats_filtered[2];
  
  // Pass to the struct cbs
  cbs.vbat_float = last_vbats_filtered[0] > 0 ? last_vbats_filtered[0] : 0;
  
  
  /* USER CODE END TIM16_IRQn 0 */
  
//  HAL_TIM_IRQHandler(&htim16);
  
  /* USER CODE BEGIN TIM16_IRQn 1 */

  /* USER CODE END TIM16_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

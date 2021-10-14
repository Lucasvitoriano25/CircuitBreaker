
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "lcd.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
CircuitBreak cbs = {IDLE, TENS, SOUND, 0, 0, 0, 0.0, 0.0, 0.0};

LCD_Values att_LCD = {0.0,0.0,0.0,0.0,NORMAL,true}; 


//LCD Pins and Ports Configurations 
LCD_PortType ports[] = {LCD_D4_GPIO_Port, LCD_D5_GPIO_Port, LCD_D6_GPIO_Port, LCD_D7_GPIO_Port};
LCD_PinType  pins[]  = {LCD_D4_Pin,       LCD_D5_Pin,       LCD_D6_Pin,       LCD_D7_Pin};
LCD_HandleTypeDef lcd;


uint16_t oc_buff[OC_BUFF_SIZE] = {0};
uint16_t oc_idx       = 0;
uint16_t oc_buff_head = 0;
uint8_t  oc_go        = 0;

uint32_t page_write_error;

float    last_ibats[3] = {0, 0, 0};
float    last_ibats_filtered[3] = {0, 0, 0};
float    last_vbats[3] = {0, 0, 0};
float    last_vbats_filtered[3] = {0, 0, 0};
float    last_ibreak     = 0.0;

uint8_t  JaPressionado   = 0;
uint32_t last_lcd_update = 0;
uint32_t beepBreakStart  = 0;
uint32_t last_psh_tick   = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void flash_write(uint32_t flash_address, float* flash_data);
float flash_read(uint32_t flash_address);
void IBreakHandler();
void beepBuzzer(uint32_t time);
void SW2_PushedHandler();
void SW3_PushedHandler();
void SW4_PushedHandler();
void SW5_PushedHandler();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
* @brief  The application entry point.
*
* @retval None
*/

int main(void)
{
  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */
  
  /* MCU Configuration----------------------------------------------------------*/
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */
  
  /* Configure the system clock */
  SystemClock_Config();
  
  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  
  /* USER CODE BEGIN 2 */
  MOSFET_OFF();

  // Perform ADC automatic self-calibration
  HAL_ADCEx_Calibration_Start(&hadc);
  
  // Init LCD
  lcd = LCD_create(ports, pins, LCD_RS_GPIO_Port, LCD_RS_Pin, LCD_E_GPIO_Port, LCD_E_Pin, LCD_4_BIT_MODE);
  
  LCD_string(&lcd, " CIRCUIT BREAK  ");
  LCD_cursor(&lcd, 1, 0);
  LCD_string(&lcd, "HW Version: 2.0");
  LCD_cursor(&lcd, 2, 0);
  LCD_string(&lcd, "FW Version: 2.0");
  LCD_cursor(&lcd, 3, 0);
  LCD_string(&lcd, "   Booting...   ");
  
  // Enable ADC DMA circular mode
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim16);
  HAL_ADC_Start_DMA(&hadc, (uint32_t*) &(cbs.ibat), 2);
  
  // Get last ibreak value from FLASH
  cbs.ibreak_float = flash_read(FLASH_USER_ADDR);
  last_ibreak      = cbs.ibreak_float;
  cbs.ibreak       = (uint16_t) (cbs.ibreak_float/ganhoINA);
  
  // Beep buzzer for 300 ms
  beepBuzzer(300);
  
  //Setting unit values of LCD
  HAL_Delay(1000);
  LCD_blinkCursor(&lcd, 1);
  LCD_cursor(&lcd, 0, 0);
  LCD_string(&lcd, "VBat  :        V");
  LCD_cursor(&lcd, 1, 0);
  LCD_string(&lcd, "IBat  :        A");
  LCD_cursor(&lcd, 2, 0);
  LCD_string(&lcd, "IBreak:        A");
  LCD_cursor(&lcd, 3, 0);
  LCD_string(&lcd, "                ");
  
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1) {
        
   IBreakHandler();
    
    // Turn BUZZER off
    if (HAL_GetTick() - beepBreakStart > 1000) {
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
    }
  
    //ATT the LCD if 100 milliseconds have passed
    if (HAL_GetTick() - last_lcd_update > 100) {
      if(cbs.state == BREAK && att_LCD.state != cbs.state) {
        LCD_cursor(&lcd, 3, 0);
        LCD_string(&lcd, " OVER CURRENT !");
        att_LCD.state = cbs.state;
      } else if(att_LCD.state != cbs.state){
        LCD_cursor(&lcd, 3, 0);
        LCD_string(&lcd, "               ");
        att_LCD.state = cbs.state;
      }
           
      // Update VBAT if the value has been updated   
     if(((int)(att_LCD.Vbat*100)) != ((int)(cbs.vbat_float*100))){
        LCD_cursor(&lcd, 0, 8);
        LCD_float(&lcd, cbs.vbat_float, 2);
        att_LCD.Vbat = cbs.vbat_float;
        LCD_cursor(&lcd, 2, 8 + cbs.digit);
      }
            
      // Update IBAT if the value has been updated
      if( !(LED_GPIO_Port->ODR && LED_Pin) && att_LCD.Debug_State){
        LCD_cursor(&lcd, 1, 8);
        LCD_float(&lcd, 0, 2);
        LCD_cursor(&lcd, 2, 8 + cbs.digit);
        att_LCD.Debug_State = false;
      }else if(((LED_GPIO_Port->ODR && LED_Pin)) && ((int)(att_LCD.Ibat*100)) != ((int)(cbs.ibat_float*100))){
        LCD_cursor(&lcd, 1, 8);
        LCD_float(&lcd, cbs.ibat_float, 2);
        att_LCD.Ibat = cbs.ibat_float;
        LCD_cursor(&lcd, 2, 8 + cbs.digit);
        att_LCD.Debug_State = true;
      }
        
    // Update IBREAK
      if(att_LCD.Ibreak != cbs.ibreak_float || att_LCD.Digit != cbs.digit){
        LCD_cursor(&lcd, 2, 8);
        LCD_float(&lcd, cbs.ibreak_float, 2);
        LCD_cursor(&lcd, 2, 8 + cbs.digit);
        att_LCD.Ibreak = cbs.ibreak_float;
        att_LCD.Digit = cbs.digit;
      }
    
      // Update last_lcd_update
      last_lcd_update = HAL_GetTick();
      //LCD_cursor(&lcd, 2, 8 + cbs.digit);

      }
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   }
  /* USER CODE END 3 */
}


/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void)
{
  
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  /**Configure the Systick interrupt time 
  */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  
  /**Configure the Systick 
  */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{
  
  ADC_ChannelConfTypeDef sConfig;
  
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  /**Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  /**Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{
  
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 59;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{
  
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 23;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000000/3200;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = htim3.Init.Period/2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  HAL_TIM_MspPostInit(&htim3);
}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 23;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 20000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */



/** 
* Enable DMA controller clock
*/
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  
  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  
}

/** Configure pins as 
* Analog 
* Input 
* Output
* EVENT_OUT
* EXTI
*/
static void MX_GPIO_Init(void)
{
  
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|LCD_RW_Pin|LCD_RS_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CMD_MOSFET_Pin|LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin 
                    |LCD_D4_Pin|LCD_E_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pins : PF0 PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  
  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /*Configure GPIO pins : PA3  */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  
  
  /*Configure GPIO pins : SW3_Pin SW4_Pin SW5_Pin */
  GPIO_InitStruct.Pin = SW3_Pin|SW4_Pin|SW5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : SW2_Pin */
  GPIO_InitStruct.Pin = SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /*Configure GPIO pins : LED_Pin LCD_RW_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = LED_Pin|LCD_RW_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /*Configure GPIO pins : CMD_MOSFET_Pin LCD_D7_Pin LCD_D6_Pin LCD_D5_Pin 
  LCD_D4_Pin LCD_E_Pin */
  GPIO_InitStruct.Pin = CMD_MOSFET_Pin|LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin 
    |LCD_D4_Pin|LCD_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  
}

/* USER CODE BEGIN 4 */

void flash_write(uint32_t flash_address, float* flash_data) {
  FLASH_EraseInitTypeDef erase_pages;
  erase_pages.PageAddress = flash_address;
  erase_pages.NbPages = 1;
  erase_pages.TypeErase = FLASH_TYPEERASE_PAGES;

  HAL_FLASH_Unlock();
  HAL_FLASHEx_Erase(&erase_pages, &page_write_error);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_address, *((uint64_t*) flash_data));
  HAL_FLASH_Lock();
}

float flash_read(uint32_t flash_address) {
  return *((float*) flash_address);
}

void IBreakHandler(){
  
  if(JaPressionado == 0){
    
    //Subtract the Maximum Current
    if( HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET) {
      SW2_PushedHandler();
      JaPressionado = 1;
      cbs.ibreak = (uint16_t) (cbs.ibreak_float / ganhoINA);
    } 
    
    //Pass the unit of the Maximum Current to the right
    if(HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_RESET) {
      SW3_PushedHandler();
      JaPressionado = 1;
      cbs.ibreak = (uint16_t) (cbs.ibreak_float / ganhoINA);
    }
    
    //Add the Maximum Current
    if(HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin) == GPIO_PIN_RESET) {
      SW4_PushedHandler();
      JaPressionado = 1;
      cbs.ibreak = (uint16_t) (cbs.ibreak_float / ganhoINA);
    }
    
    //Pass the unit of the Maximum Current to the left
    if(HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin) == GPIO_PIN_RESET) {
      SW5_PushedHandler();
      JaPressionado = 1;
      cbs.ibreak = (uint16_t) (cbs.ibreak_float / ganhoINA);
    }
    
    // Check if ibreak changed
    if (cbs.ibreak_float != last_ibreak) {
      // Update last_ibreak and EEPROM value
      last_ibreak = cbs.ibreak_float;
      flash_write(FLASH_USER_ADDR, &last_ibreak);
    }
    
    // Check if change mute state buttons was pushed
    if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET &&
       HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_SET   &&
       HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin) == GPIO_PIN_RESET &&
       HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin) == GPIO_PIN_SET)
    {
      cbs.buzzer = !cbs.buzzer;
      
      if (cbs.buzzer == SOUND) {
        beepBuzzer(300);
      }
      
      JaPressionado = 1;
    }
  }
  
  //Reset LCD to initial values
  if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_SET   &&
     HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_RESET &&
     HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin) == GPIO_PIN_SET   &&
     HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin) == GPIO_PIN_RESET)
  {
    LCD_init(&lcd);
    LCD_blinkCursor(&lcd, 1);
    LCD_cursor(&lcd, 0, 0);
    LCD_string(&lcd, "VBat:          V");
    LCD_cursor(&lcd, 1, 0);
    LCD_string(&lcd, "IBat:          A");
    LCD_cursor(&lcd, 2, 0);
    LCD_string(&lcd, "IBreak:        A");
    LCD_cursor(&lcd, 3, 0);
    LCD_string(&lcd, "                ");
  }
  //Checks if all butons are in default state 
  if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_SET &&
     HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_SET &&
     HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin) == GPIO_PIN_SET &&
     HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin) == GPIO_PIN_SET)
  {
    JaPressionado = 0;
  }
  
}

void beepBuzzer(uint32_t time) {
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(time);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
}

//Tira 1 de onde está o ponteiro 
void SW2_PushedHandler() {
  switch (cbs.digit) {
    case HUNDREDS:
      if (cbs.ibreak_float >= 100) {
        cbs.ibreak_float -= 100;
      }
    break;
      
    case TENS:
      if (cbs.ibreak_float >= 10) {
        cbs.ibreak_float -= 10;
      }
    break;
      
    case UNITS:
      if (cbs.ibreak_float >= 1) {
        cbs.ibreak_float -= 1;
      }
    break;
      
    case TENTHS:
      if (cbs.ibreak_float >= 0.1) {
        cbs.ibreak_float -= 0.1;
      }
    break;
      
    case HUNDREDTHS:
      if (cbs.ibreak_float >= 0.01) {
        cbs.ibreak_float -= 0.01;
      }
    break;
  }
  
  if (cbs.buzzer == SOUND) {
    beepBuzzer(20);
  }
}

//Desloca o ponteiro para a direita 
void SW3_PushedHandler() {
  switch (cbs.digit) {
    case HUNDREDS:
      cbs.digit = TENS;
    break;
    
    case TENS:
      cbs.digit = UNITS;
    break;
    
    case UNITS:
      cbs.digit = TENTHS;
    break;
    
    case TENTHS:
      cbs.digit = HUNDREDTHS;
    break;
    
    case HUNDREDTHS:
      cbs.digit = HUNDREDS;
    break;
  }
  
  if (cbs.buzzer == SOUND) {
    beepBuzzer(20);
  }
}

//Adiciona +1 onde está o ponteiro 
void SW4_PushedHandler() {
  switch (cbs.digit) {
    case HUNDREDS:
      if (cbs.ibreak_float <= 60) {
        cbs.ibreak_float += 100;
      }
    break;
    
    case TENS:
      if (cbs.ibreak_float <= 150) {
        cbs.ibreak_float += 10;
      }
    break;
    
    case UNITS:
      if (cbs.ibreak_float <= 159) {
        cbs.ibreak_float += 1;
      }
    break;
    
    case TENTHS:
      if (cbs.ibreak_float <= 159.9) {
        cbs.ibreak_float += 0.1;
      }
    break;
    
    case HUNDREDTHS:
      if (cbs.ibreak_float <= 159.99) {
        cbs.ibreak_float += 0.01;
      }
    break;
  }
  
  if (cbs.buzzer == SOUND) {
    beepBuzzer(20);
  }
}

//Desloca o ponteiro para a esquerda
void SW5_PushedHandler() {
  switch (cbs.digit) {
    case HUNDREDS:
      cbs.digit = HUNDREDTHS;
    break;
    
    case TENS:
      cbs.digit = HUNDREDS;
    break;
    
    case UNITS:
      cbs.digit = TENS;
    break;
    
    case TENTHS:
      cbs.digit = UNITS;
    break;
    
    case HUNDREDTHS:
      cbs.digit = TENTHS;
    break;
  }
  
  if (cbs.buzzer == SOUND) {
    beepBuzzer(20);
  }
}
/* USER CODE END 4 */

/**
* @brief  This function is executed in case of error occurrence.
* @param  file: The file name as string.
* @param  line: The line in file as a number.
* @retval None
*/
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

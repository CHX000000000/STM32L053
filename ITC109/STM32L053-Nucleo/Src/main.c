
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
  * COPYRIGHT(c) 2021 STMicroelectronics
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
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim22;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t mode=0,b_mode,led[3],led_l,t;
uint8_t value[5],h[12]={3,2,1,12,11,10,9,8,7,6,5,4};
double value_g[3];
_Bool led_c,c7=0;
uint32_t tick;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM22_Init(void);
static void MX_TIM2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void adc_(void);
void sw(void);
void m0(void);
void m1(void);
void m2(void);
void m3(void);
void m4(void);
void m5(void);
void m6(void);
void m7(void);
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
	//uint16_t main_i = 0;
	//uint8_t sw_stage = 0;
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
  MX_ADC_Init();
  MX_TIM22_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	/*
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
	*/
	GPIOB->BRR=0x03;
	GPIOC->ODR=0x1fff;
	
	HAL_TIM_PWM_Start(&htim22,TIM_CHANNEL_1);
	TIM22->CCR1=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		sw();
		
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

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV64;
  hadc.Init.Resolution = ADC_RESOLUTION_8B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = ENABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
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

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 31;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM22 init function */
static void MX_TIM22_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim22.Instance = TIM22;
  htim22.Init.Prescaler = 106;
  htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim22.Init.Period = 99;
  htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim22) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim22, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim22) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim22, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim22, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim22);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 
                           PC4 PC5 PC6 PC7 
                           PC8 PC9 PC10 PC11 
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void adc_()
{
	HAL_ADC_Start(&hadc);
	for(uint8_t i=0; i<5; i++)
	{
		HAL_ADC_PollForConversion(&hadc,10);
		value[i]=HAL_ADC_GetValue(&hadc);
	}
	HAL_ADC_Stop(&hadc);
	float num;
	for(uint8_t i=0; i<3; i++)
	{
		num=0;
		for(uint8_t ii=0; ii<20; ii++)
		{
			num++;
			if(value[i]<=num*4.25+85)
			{
				value_g[i]=(num*0.1)-1;
				break;
			}
		}
	}
}
void sw()
{
	mode=((GPIOB->IDR)&0x1C)>>2;
	switch (mode)
	{
		case 0: 
			m0();
		break;
		case 1: 
			m1();
		break;
		case 2: 
			m2();
		break;
		case 3: 
			m3();
		break;
		case 4: 
			m4();
		break;
		case 5: 
			m5();
		break;
		case 6: 
			m6();
		break;
		case 7: 
			m7();
		break;
		
	}
}
void m0()
{
	if(mode!=b_mode)
	{
		b_mode=mode;
		HAL_TIM_Base_Stop_IT(&htim2);
		GPIOB->BRR=0x03;
		GPIOC->ODR=0x1fff;
	}
}
void m1()
{
	if(mode!=b_mode)
	{
		b_mode=mode;
		GPIOB->BRR=0x03;
		GPIOC->ODR=0x1fff;
		HAL_TIM_Base_Start_IT(&htim2);
		led[0]=6;
		led[1]=6;
		led[2]=0;
		tick=HAL_GetTick();
	}
	if(HAL_GetTick()-tick>=1000)
	{
		if(++led[0]==12)
		{
			if(++led[1]==13) led[1]=1;
		}
		if(led[0]==13) led[0]=1;
		tick=HAL_GetTick();
	}
}
void m2()
{
	if(mode!=b_mode)
	{
		b_mode=mode;
		GPIOB->BRR=0x03;
		GPIOC->ODR=~0x1ffe;
		HAL_TIM_Base_Start_IT(&htim2);
		tick=HAL_GetTick();
		led_c=0;
		t=0;
	}
	if(HAL_GetTick()-tick>=200)
	{
		t++;
		tick=HAL_GetTick();
		if(t<5) led_l+=2;
		else led_l-=2;
		if(t>=10)
		{
			t=0;
			led_c=~led_c;
			led_l=0;
		}
	}
}
void m3()
{
	if(mode!=b_mode)
	{
		b_mode=mode;
		HAL_TIM_Base_Stop_IT(&htim2);
		GPIOB->BRR=0x03;
		GPIOC->ODR=0x1fff;
		tick=HAL_GetTick();
	}
	if(HAL_GetTick()-tick>=100)
	{
		adc_();
		if(value_g[0]<-0.7)
		{
			GPIOB->BRR=0x03;
			GPIOC->ODR=0x1fff;
			GPIOB->BSRR=0x01;
			GPIOC->BRR=1<<3;
		}else if(value_g[0]<-0.3)
		{
			GPIOB->BRR=0x03;
			GPIOC->ODR=0x1fff;
			GPIOB->BSRR=0x02;
			GPIOC->BRR=1<<3;
		}else if(value_g[0]< 0.3)
		{
			GPIOB->BRR=0x03;
			GPIOC->ODR=0x1fff;
			GPIOB->BSRR=0x02;
			GPIOC->BRR=1<<0;
		}else if(value_g[0]< 0.7)
		{
			GPIOB->BRR=0x03;
			GPIOC->ODR=0x1fff;
			GPIOB->BSRR=0x02;
			GPIOC->BRR=1<<9;
		}else
		{
			GPIOB->BRR=0x03;
			GPIOC->ODR=0x1fff;
			GPIOB->BSRR=0x01;
			GPIOC->BRR=1<<9;
		}
		tick=HAL_GetTick();
	}
}
void m4()
{
	if(mode!=b_mode)
	{
		b_mode=mode;
		HAL_TIM_Base_Stop_IT(&htim2);
		GPIOB->BRR=0x03;
		GPIOC->ODR=0x1fff;
		tick=HAL_GetTick();
	}
	if(HAL_GetTick()-tick>=100)
	{
		adc_();
		float g=value[3]*3.3/255;
		if(g<0.8)
		{
			GPIOB->BRR=0x03;
			GPIOC->ODR=0x1fff;
			GPIOB->BSRR=0x01;
			GPIOC->BRR=1<<9;
		}else if(g<1.2)
		{
			GPIOB->BRR=0x03;
			GPIOC->ODR=0x1fff;
			GPIOB->BSRR=0x02;
			GPIOC->BRR=1<<9;
		}else if(g<2)
		{
			GPIOB->BRR=0x03;
			GPIOC->ODR=0x1fff;
			GPIOB->BSRR=0x02;
			GPIOC->BRR=1<<0;
		}else if(g<2.4)
		{
			GPIOB->BRR=0x03;
			GPIOC->ODR=0x1fff;
			GPIOB->BSRR=0x02;
			GPIOC->BRR=1<<3;
		}else
		{
			GPIOB->BRR=0x03;
			GPIOC->ODR=0x1fff;
			GPIOB->BSRR=0x01;
			GPIOC->BRR=1<<3;
		}
		tick=HAL_GetTick();
	}
}
void m5()
{
	if(mode!=b_mode)
	{
		b_mode=mode;
		HAL_TIM_Base_Stop_IT(&htim2);
		GPIOB->BRR=0x03;
		GPIOC->ODR=0x1fff;
		tick=HAL_GetTick();
	}
	if(HAL_GetTick()-tick>=100)
	{
		adc_();
		double xy=sqrt(value_g[0]*value_g[0]+value_g[1]*value_g[1]),theta=atan2(value_g[1],value_g[0]);
		theta = theta / 3.1415926 * 180;
		GPIOB->BRR=0x03;
		GPIOC->ODR=0x1fff;
		if(xy<0.3)
		{
			GPIOB->BSRR=0x02;
			GPIOC->BRR=1<<0;
		}else if(xy<0.7)
		{
			GPIOB->BSRR=0x02;
			if(theta<15) GPIOC->BRR=1<<3;
			else if(theta<45) GPIOC->BRR=1<<2;
			else if(theta<75) GPIOC->BRR=1<<1;
			else if(theta<105) GPIOC->BRR=1<<12;
			else if(theta<135) GPIOC->BRR=1<<11;
			else if(theta<165) GPIOC->BRR=1<<10;
			else if(theta<195) GPIOC->BRR=1<<9;
			else if(theta<225) GPIOC->BRR=1<<8;
			else if(theta<255) GPIOC->BRR=1<<7;
			else if(theta<285) GPIOC->BRR=1<<6;
			else if(theta<315) GPIOC->BRR=1<<5;
			else if(theta<345) GPIOC->BRR=1<<4;
		}else
		{
			GPIOB->BSRR=0x01;
			if(theta<15) GPIOC->BRR=1<<3;
			else if(theta<45) GPIOC->BRR=1<<2;
			else if(theta<75) GPIOC->BRR=1<<1;
			else if(theta<105) GPIOC->BRR=1<<12;
			else if(theta<135) GPIOC->BRR=1<<11;
			else if(theta<165) GPIOC->BRR=1<<10;
			else if(theta<195) GPIOC->BRR=1<<9;
			else if(theta<225) GPIOC->BRR=1<<8;
			else if(theta<255) GPIOC->BRR=1<<7;
			else if(theta<285) GPIOC->BRR=1<<6;
			else if(theta<315) GPIOC->BRR=1<<5;
			else if(theta<345) GPIOC->BRR=1<<4;
		}
		tick=HAL_GetTick();
	}
}
void m6()
{
	if(mode!=b_mode)
	{
		b_mode=mode;
		GPIOB->BRR=0x03;
		GPIOC->ODR=0x1fff;
		HAL_TIM_Base_Start_IT(&htim2);
		tick=HAL_GetTick();
		t=1;
	}
	if(HAL_GetTick()-tick>=200)
	{
		TIM22->CCR1=0;
		adc_();
		double xy=sqrt((value_g[0]*value_g[0])+(value_g[1]*value_g[1])),theta=atan2(value_g[1],value_g[0]);
		theta = theta / 3.1415926 * 180;
		t++;
		if(xy>=0.3)
		{
			
			for(uint8_t i=1;i<13;i++)
			{
				if(theta<=(i*30)-15)
				{
					if(h[i]==t) TIM22->CCR1=50;
					if(xy<0.7)
					{
						led[1]=i;
					}else
					{
						led[0]=i;
					}
				}
			}
		}
		tick=HAL_GetTick();
	}
}
void m7()
{
	if(mode!=b_mode)
	{
		b_mode=mode;
		GPIOB->BRR=0x03;
		GPIOC->ODR=0x1fff;
		HAL_TIM_Base_Stop_IT(&htim2);
		tick=HAL_GetTick();
	}
	if(HAL_GetTick()-tick>=100)
	{
		adc_();
		double xy=sqrt((value_g[0]*value_g[0])+(value_g[1]*value_g[1])),theta=atan2(value_g[1],value_g[0]);
		theta = theta / 3.1415926 * 180;
		c7=0;
		if(xy<0.3)
		{
			c7=1;
		}else if(xy<0.7)
		{
			GPIOB->BRR=0x03;
			GPIOC->ODR=0x1fff;
		}else
		{
			GPIOB->BRR=0x03;
			GPIOC->ODR=0x1fff;
			GPIOB->BSRR=0x03;
			if(theta<15) GPIOC->BRR=(1<<3)|0x01;
			else if(theta<45) GPIOC->BRR=(1<<2)|0x01;
			else if(theta<75) GPIOC->BRR=(1<<1)|0x01;
			else if(theta<105) GPIOC->BRR=(1<<12)|0x01;
			else if(theta<135) GPIOC->BRR=(1<<11)|0x01;
			else if(theta<165) GPIOC->BRR=(1<<10)|0x01;
			else if(theta<195) GPIOC->BRR=(1<<9)|0x01;
			else if(theta<225) GPIOC->BRR=(1<<8)|0x01;
			else if(theta<255) GPIOC->BRR=(1<<7)|0x01;
			else if(theta<285) GPIOC->BRR=(1<<6)|0x01;
			else if(theta<315) GPIOC->BRR=(1<<5)|0x01;
			else if(theta<345) GPIOC->BRR=(1<<4)|0x01;
		}
		tick=HAL_GetTick();
	}
}
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim2)
{
	
}
*/
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

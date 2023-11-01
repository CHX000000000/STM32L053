
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
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#include "adc.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
extern ADC_HandleTypeDef hadc;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t hour = 0;
uint8_t min = 0;
uint8_t sec = 0;
uint8_t mode=0,b_mode=0,t1=0,s2=0,n=0;
uint16_t ain[4],c1=0,RPM,ni=0,nii=0,bn=10,nnn,cnn=0;
uint32_t t[2];
char str[32],stt[9][10]={{"  CENTER  "},
												 {"    UP    "},
												 {" RIGHT_UP "},
												 {"  RIGHT   "},
												 {"RIGHT_DOWN"},
												 {"   DOWN   "},
												 {"LEFT_DOWN "},
												 {"   LEFT   "},
												 {"  LEFT_UP "}};											 
_Bool flag_R[2]={1,1},ct1=1,c2=1,c22=0,c222=0,c0=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Set_BCD_Hour(char);
void Set_BCD_Min(char);
void Set_BCD_Sec(char);
char Get_BCD_Hour(void);
char Get_BCD_Min(void);
char Get_BCD_Sec(void);

void adc()
{
	for(uint8_t i=0;i<4;i++)
	{
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc,10);
		ain[i]=HAL_ADC_GetValue(&hadc);
	}
}
void m0()
{
	if(b_mode!=mode)
	{
		b_mode=mode;
		GPIOC->ODR=~0x80;
		GPIOB->ODR=0x00;
		TIM22->CCR1=0;
		OLED_Start_Page_PageAddressing(0);
		OLED_Start_Column_PageAddressing(0);
		t[0]=HAL_GetTick();
		c0=0;
	}
	if(c0)
	{
		HAL_RTC_GetDate(&hrtc,&sDate,FORMAT_BIN);
		HAL_RTC_GetTime(&hrtc,&sTime,FORMAT_BIN);
		sprintf(str,"System RTC: %02d:%02d:%02d",sTime.Hours,sTime.Minutes,sTime.Seconds);
		OLED_ShowRomString(2,str);
	}
	if(HAL_GetTick()-t[0]>=200)
	{
		c0=1;
		adc();
		OLED_ShowRomString(0,"NTSC_48T: Electronics");
		OLED_ShowRomString(1,"Competitor: 000000002");
		sprintf(str,"AIN0: %04d[mV]",ain[0]*3300/4095);
		OLED_ShowRomString(3,str);
		sprintf(str,"AIN1: %04d[mV]",ain[1]*3300/4095);
		OLED_ShowRomString(4,str);
		sprintf(str,"AIN2: %04d[mV]",ain[2]*3300/4095);
		OLED_ShowRomString(5,str);
		sprintf(str,"AIN3: %04d[mV]",ain[3]*3300/4095);
		OLED_ShowRomString(6,str);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==0)
		{
			OLED_ShowRomString(7,"RPM:    BLANKED");
		}
		else
		{
			OLED_ShowRomString(7,"RPM:  NOT_BLANKED");
		}
		t[0]=HAL_GetTick();
	}
}
void m1()
{
	if(b_mode!=mode)
	{
		b_mode=mode;
		GPIOC->ODR=~0x40;
		GPIOB->ODR=0x00;
		TIM22->CCR1=0;
		OLED_Start_Page_PageAddressing(0);
		OLED_Start_Column_PageAddressing(0);
		t[0]=t[1]=HAL_GetTick();
		OLED_ShowRomString(6,"Relay1: OFF");
		OLED_ShowRomString(7,"Relay2: OFF");
	}
	if(HAL_GetTick()-t[0]>=200)
	{
		adc();
		OLED_ShowRomString(0,"Power GenEration Info");
		if(ain[1]>3000) TIM22->CCR1+=5;
		else if(ain[1]<1000&&TIM22->CCR1>=5) TIM22->CCR1-=5;
		else if(ain[1]<1000) TIM22->CCR1=0;
		else if(ain[0]>3000) TIM22->CCR1+=1;
		else if(ain[0]<1000&&TIM22->CCR1>0) TIM22->CCR1-=1;
		if(TIM22->CCR1>100) TIM22->CCR1=100;
		sprintf(str,"PWM1: %3d  [%%]",TIM22->CCR1);
		OLED_ShowRomString(1,str);
		sprintf(str,"RPM:  %4d [RPM]",RPM*60);
		OLED_ShowRomString(2,str);
		uint16_t pp=RPM*60/TIM22->CCR1;
		if(TIM22->CCR1==0) pp=0;
		sprintf(str,"Rate: %4d [RPM/PWM1]",pp);
		OLED_ShowRomString(3,str);
		sprintf(str,"Vout: %4d [mV]",ain[2]*3300/4095);
		OLED_ShowRomString(4,str);
		sprintf(str,"Iout: %5.1f[mA]",(float)ain[3]*3300/4095/11);
		OLED_ShowRomString(5,str);
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10)==0&&flag_R[0])
		{
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)) OLED_ShowRomString(6,"Relay1: ON");
			else OLED_ShowRomString(6,"Relay1: OFF");
			flag_R[0]=0;
		}else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10)==1)
		{
			flag_R[0]=1;
		}
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)==0&&flag_R[1])
		{
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)) OLED_ShowRomString(7,"Relay2: ON");
			else OLED_ShowRomString(7,"Relay2: OFF");
			flag_R[1]=0;
		}else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)==1)
		{
			flag_R[1]=1;
		}
		t[0]=HAL_GetTick();
	}
	if(HAL_GetTick()-t[1]<500)
	{		
		if(ct1&&HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==0)
		{
			c1++;
			ct1=0;
		}else if(!ct1&&HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2))
		{
			ct1=1;
		}
	}else
	{
		t[1]=HAL_GetTick();
		RPM=c1;
		c1=0;
	}
}
void m2()
{
	if(b_mode!=mode)
	{
		b_mode=mode;
		GPIOC->ODR=~0x20;
		GPIOB->ODR=0x00;
		OLED_Start_Page_PageAddressing(0);
		OLED_Start_Column_PageAddressing(0);
		t[0]=HAL_GetTick();
		OLED_ShowLogo();
	}
	int i=0;
	HAL_RTC_GetDate(&hrtc,&sDate,FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc,&sTime,FORMAT_BIN);
	OLED_Start_Page_PageAddressing(0);
	OLED_Start_Column_PageAddressing(0);
	sprintf(str,"%02d:%02d:%02d",sTime.Hours,sTime.Minutes,sTime.Seconds);
	while(str[i]!='\0') OLED_ShowRomChar(str[i++]);
	if(HAL_GetTick()-t[0]>=200)
	{
		adc();
		OLED_Start_Page_PageAddressing(1);
		OLED_Start_Column_PageAddressing(0);
		i=0;
		if(ain[0]>3000&&ain[1]>3000)
		{
			while(i<10) OLED_ShowRomChar(stt[2][i++]);
			n=2;
		}			
		else if(ain[0]>3000&&ain[1]<1000)
		{
			while(i<10) OLED_ShowRomChar(stt[4][i++]);
			n=4;
		}	
		else if(ain[0]<1000&&ain[1]>3000)
		{
			while(i<10) OLED_ShowRomChar(stt[8][i++]);
			n=8;
		}
		else if(ain[0]<1000&&ain[1]<1000)
		{
			while(i<10) OLED_ShowRomChar(stt[6][i++]);
			n=6;
		}
		else if(ain[0]>3000)
		{
			while(i<10) OLED_ShowRomChar(stt[3][i++]);
			n=3;
		}
		else if(ain[1]>3000)
		{
			while(i<10) OLED_ShowRomChar(stt[1][i++]);
			n=1;
		}
		else if(ain[0]<1000)
		{
			while(i<10) OLED_ShowRomChar(stt[7][i++]);
			n=7;
		}
		else if(ain[1]<1000)
		{
			while(i<10) OLED_ShowRomChar(stt[5][i++]);
			n=5;
		}
		else if(ain[1]>1000&&ain[0]>1000&&ain[0]<3000&&ain[1]<3000)
		{
			while(i<10) OLED_ShowRomChar(stt[0][i++]);
			n=0;
		}
		if(n!=0&&bn!=n)
		{
			bn=n;
			if(c2==1) 
			{
				nnn=ni=nii=n;
				c2=0;
				nii++;
				if(nii>8) nii=0;
				nnn--;
				if(nnn<0) nnn=8;
			}
			if(n==nii&&s2==0)
			{
					s2=1;
					nii++;
					if(nii>8) nii=0;			
			}
			if(s2==0&&n==nnn)
			{
					s2=2;
					nnn--;
					if(nnn<0) nnn=8;
					nii=nnn;				
			}
			cnn+=n;
			if(cnn>=36) c22=1;
			if(n==nii&&s2==1)
			{
				nii++;
				if(n!=ni) c222=1;
				if(nii>8) nii=0;
				if(n==ni&&c222) c22=1;
			}
			else if(n==nii&&s2==2)
			{
				nii--;
				if(n!=ni) c222=1;
				if(nii<0) nii=8;
				if(n==ni&&c222) c22=1;
			}				
		}else if(n==0)
		{
			c2=1;
			s2=0;
			c22=0;
			c222=0;
			cnn=0;
		}
    OLED_Start_Page_PageAddressing(2);
		OLED_Start_Column_PageAddressing(0);
		i=0;
		if(s2==0)
		{
			OLED_ShowRomChar('N');
			OLED_ShowRomChar('O');
			OLED_ShowRomChar('N');
			OLED_ShowRomChar('E');
		}
		else if(s2==1&&c22)
		{
			OLED_ShowRomChar(' ');
			OLED_ShowRomChar('C');
			OLED_ShowRomChar('W');
			OLED_ShowRomChar(' ');
		}
		else if(s2==2&&c22)
		{
			OLED_ShowRomChar(' ');
			OLED_ShowRomChar('C');
			OLED_ShowRomChar('C');
			OLED_ShowRomChar('W');
		}
		t[0]=HAL_GetTick();
	}
}
void m()
{
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
	}
}
void s()
{
	if(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11))
	{
		mode++;
		if(mode>=3)mode=0;
		while(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11))
		{
			m();
		}
		HAL_Delay(40);
	}
}

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
  MX_TIM22_Init();
  MX_RTC_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim22);
	HAL_TIM_PWM_Start(&htim22 , TIM_CHANNEL_1);


	HAL_Delay(100);	//Wait for RTC Init
	
	HAL_GPIO_WritePin(GPIOC , GPIO_PIN_8 , GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC , GPIO_PIN_7 , GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC , GPIO_PIN_6 , GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC , GPIO_PIN_5 , GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC , GPIO_PIN_4 , GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB , GPIO_PIN_1 , GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB , GPIO_PIN_0 , GPIO_PIN_RESET);

	OLED_Init();
	HAL_Delay(1000);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		s();
		m();
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)));
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Configure LSE Drive Capability 
    */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/* USER CODE BEGIN 4 */
void Set_BCD_Hour(char hour)
{
  RTC_TimeTypeDef sTime;	
	HAL_RTC_GetTime(&hrtc , &sTime , RTC_FORMAT_BCD);
	sTime.Hours = hour;
	HAL_RTC_SetTime(&hrtc , &sTime , RTC_FORMAT_BCD);
}


void Set_BCD_Min(char min)
{
  RTC_TimeTypeDef sTime;	
	HAL_RTC_GetTime(&hrtc , &sTime , RTC_FORMAT_BCD);
	sTime.Minutes = min;
	HAL_RTC_SetTime(&hrtc , &sTime , RTC_FORMAT_BCD);
}	

void Set_BCD_Sec(char sec)
{
  RTC_TimeTypeDef sTime;	
	HAL_RTC_GetTime(&hrtc , &sTime , RTC_FORMAT_BCD);
	sTime.Seconds = sec;
	HAL_RTC_SetTime(&hrtc , &sTime , RTC_FORMAT_BCD);
}



char Get_BCD_Hour(void)
{
  RTC_TimeTypeDef sTime;	
	char hour;	
	HAL_RTC_GetTime(&hrtc , &sTime , RTC_FORMAT_BCD);
	hour = sTime.Hours;
	return hour;
}


char Get_BCD_Min(void)
{
  RTC_TimeTypeDef sTime;	
	char min;	
	HAL_RTC_GetTime(&hrtc , &sTime , RTC_FORMAT_BCD);
	min = sTime.Minutes;
	return min;		
}	

char Get_BCD_Sec(void)
{
  RTC_TimeTypeDef sTime;	
	char sec;	
	HAL_RTC_GetTime(&hrtc , &sTime , RTC_FORMAT_BCD);
	sec = sTime.Seconds;
	return sec;	
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

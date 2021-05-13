/**
  ******************************************************************************
  * @file    ADC/ADC_RegularConversion_Polling/Src/main.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    26-February-2014
  * @brief   This example describes how to use Polling mode to convert data.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#include <stdio.h>

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */
	
/** @addtogroup ADC_RegularConversion_Polling
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* ADC handler declaration */
ADC_HandleTypeDef    AdcHandle1, AdcHandle2, AdcHandle3;
ADC_ChannelConfTypeDef adcConfig1, adcConfig2, adcConfig3;
ADC_ChannelConfTypeDef sConfig;

/* Variable used to get converted value */
__IO uint32_t uhADCxRight;
__IO uint32_t uhADCxBottom;
__IO uint32_t uhADCxLeft;	

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

extern UART_HandleTypeDef UartHandle1, UartHandle2;

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
	
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_UART_Transmit(&UartHandle1, (uint8_t *)&ch, 1, 0xFFFF); 

	return ch;
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	/* STM32F4xx HAL library initialization:
	   - Configure the Flash prefetch, instruction and Data caches
	   - Configure the Systick to generate an interrupt each 1 msec
	   - Set NVIC Group Priority to 4
	   - Global MSP (MCU Support Package) initialization
	 */
	HAL_Init();

	/* Configure the system clock to 144 Mhz */
	SystemClock_Config();

	/* Configure LED3 */
	BSP_LED_Init(LED3);
	
	BSP_COM1_Init();

	//##-1- Configure the ADC peripheral #######################################*/
	AdcHandle1.Instance          = ADC3;   // ADC 3번 - Left
  
	AdcHandle1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	AdcHandle1.Init.Resolution = ADC_RESOLUTION12b;
	AdcHandle1.Init.ScanConvMode = DISABLE;
	// Mode 설정
	AdcHandle1.Init.ContinuousConvMode = DISABLE;
	AdcHandle1.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle1.Init.NbrOfDiscConversion = 0;  
	AdcHandle1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	AdcHandle1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	AdcHandle1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	AdcHandle1.Init.NbrOfConversion = 1;
	//DMA(Direct Memory Access)
	AdcHandle1.Init.DMAContinuousRequests = DISABLE;
	AdcHandle1.Init.EOCSelection = DISABLE;

	HAL_ADC_Init(&AdcHandle1);//ADC Initialized

	/*##-1- Configure ADC regular channel ######################################*/ 
	adcConfig1.Channel = ADC_CHANNEL_11; //채널 설정
	adcConfig1.Rank = 1;
	adcConfig1.SamplingTime = ADC_SAMPLETIME_480CYCLES; //샘플링 주기 설정
	adcConfig1.Offset = 0;

	HAL_ADC_ConfigChannel(&AdcHandle1, &adcConfig1);
		
		
		
		
  //##-2- Configure the ADC peripheral #######################################*/
	AdcHandle2.Instance          = ADC2;   // ADC 2번 - Right

	AdcHandle2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	AdcHandle2.Init.Resolution = ADC_RESOLUTION12b;
	AdcHandle2.Init.ScanConvMode = DISABLE;
	AdcHandle2.Init.ContinuousConvMode = DISABLE;
	AdcHandle2.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle2.Init.NbrOfDiscConversion = 0;  
	AdcHandle2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	AdcHandle2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	AdcHandle2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	AdcHandle2.Init.NbrOfConversion = 1;
	AdcHandle2.Init.DMAContinuousRequests = DISABLE;
	AdcHandle2.Init.EOCSelection = DISABLE;

	HAL_ADC_Init(&AdcHandle2);

	/*##-2- Configure ADC regular channel ######################################*/ 
	adcConfig2.Channel = ADC_CHANNEL_14;
	adcConfig2.Rank = 1;
	adcConfig2.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	adcConfig2.Offset = 0;	

	HAL_ADC_ConfigChannel(&AdcHandle2, &adcConfig2);
	
	
	
	
	 //##-3- Configure the ADC peripheral #######################################*/
	AdcHandle3.Instance          = ADC1;   // ADC 1번 - Bottom

	AdcHandle3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	AdcHandle3.Init.Resolution = ADC_RESOLUTION12b;
	AdcHandle3.Init.ScanConvMode = DISABLE;
	AdcHandle3.Init.ContinuousConvMode = DISABLE;
	AdcHandle3.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle3.Init.NbrOfDiscConversion = 0;  
	AdcHandle3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	AdcHandle3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	AdcHandle3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	AdcHandle3.Init.NbrOfConversion = 1;
	AdcHandle3.Init.DMAContinuousRequests = DISABLE;
	AdcHandle3.Init.EOCSelection = DISABLE;

	HAL_ADC_Init(&AdcHandle3);

	/*##-3- Configure ADC regular channel ######################################*/ 
	adcConfig3.Channel = ADC_CHANNEL_15;
	adcConfig3.Rank = 1;
	adcConfig3.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	adcConfig3.Offset = 0;	
	HAL_ADC_ConfigChannel(&AdcHandle3, &adcConfig3);
				
	/* Infinite loop */
	while(1)
	{
		printf("\r\nStudent Number : 202100000	");
		HAL_ADC_Start(&AdcHandle1);
		uhADCxLeft = HAL_ADC_GetValue(&AdcHandle1);
		HAL_ADC_PollForConversion(&AdcHandle1, 0xFF);	
		if(uhADCxLeft >2000) uhADCxLeft= 2000;
		else if(uhADCxLeft<100) uhADCxLeft = 100;
		printf("\r\nIR sensor Left = %d", uhADCxLeft);
	
		
		HAL_ADC_Start(&AdcHandle2);
		uhADCxRight = HAL_ADC_GetValue(&AdcHandle2);
		HAL_ADC_PollForConversion(&AdcHandle2, 0xFF);
		if(uhADCxRight >2000) uhADCxRight= 2000;
		else if(uhADCxRight<100) uhADCxRight = 100;
		printf("\r\nIR sensor Right = %d", uhADCxRight);

		
		HAL_ADC_Start(&AdcHandle3);
		//현재 ADC 값을 읽어온다.
		uhADCxBottom = HAL_ADC_GetValue(&AdcHandle3);
		HAL_ADC_PollForConversion(&AdcHandle3, 0xFF);
		if(uhADCxBottom >2000) uhADCxBottom= 2000;
		else if(uhADCxBottom<100)	uhADCxBottom = 100;
		printf("\r\nIR sensor Bottom = %d", uhADCxBottom);
		
		HAL_Delay(500);
	}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 144000000
  *            HCLK(Hz)                       = 144000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 288
  *            PLL_P                          = 2
  *            PLL_Q                          = 6
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable Power Control clock */
	__PWR_CLK_ENABLE();

	/* The voltage scaling allows optimizing the power consumption when the device is 
	 clocked below the maximum system frequency, to update the voltage scaling value 
	 regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 288;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 6;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
	 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
	/* Turn LED3 on */
	BSP_LED_On(LED3);
	while(1)
	{
	}
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
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}

#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

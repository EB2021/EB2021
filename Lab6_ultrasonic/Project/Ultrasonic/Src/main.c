/**
  ******************************************************************************
  * @file    TIM/TIM_InputCapture/Src/main.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    26-February-2014
  * @brief   This example shows how to use the TIM peripheral to measure only 
  *          the frequency  of an external signal.
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
#include "cmsis_os.h"


/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/**
  * @}
  */
	
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Timer handler declaration */
TIM_HandleTypeDef      TimHandle1, TimHandle2;
TIM_OC_InitTypeDef sConfig;
	
/* Timer Input Capture Configuration Structure declaration */
TIM_IC_InitTypeDef     sICConfig;

/* Captured Values */
uint32_t               uwIC2Value1 = 0;
uint32_t               uwIC2Value2 = 0;
uint32_t               uwDiffCapture1 = 0;
	
uint32_t               uwIC2Value3 = 0;
uint32_t               uwIC2Value4 = 0;
uint32_t               uwDiffCapture2 = 0;

uint32_t               uwIC2Value5 = 0;
uint32_t               uwIC2Value6= 0;
uint32_t               uwDiffCapture3 = 0;

/* Capture index */
uint32_t               uhCaptureIndex = 0;

/* Frequency Value */
uint32_t               uwFrequency = 0;


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

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
	/* Compute the prescaler value to have TIM3 counter clock equal to 1 MHz */
	uint32_t uwPrescalerValue = 0;

	/* STM32F4xx HAL library initialization:
	   - Configure the Flash prefetch, instruction and Data caches
	   - Configure the Systick to generate an interrupt each 1 msec
	   - Set NVIC Group Priority to 4
	   - Global MSP (MCU Support Package) initialization
	 */
	HAL_Init();
	
	/* Configure the system clock to have a system clock = 180 Mhz */
	SystemClock_Config();
	
	BSP_COM1_Init();
	
	/* Configure LED3 */
	BSP_LED_Init(LED3);
	
	uwPrescalerValue = ((SystemCoreClock / 2) / 1000000) - 1;	
	
	
	/*============= ① 그림 5. TIMHandle / Input Capture Configuration 부분 ================*/
	
	// TIM_HandleTypeDef 구조체 선언
	// Timer 3번
	TimHandle1.Instance = 

	// 카운트 횟수 0xFFFF 65535
	TimHandle1.Init.Period        = 
	TimHandle1.Init.Prescaler     = 
	
	// 프리스케일 사용자 정의
	TimHandle1.Init.ClockDivision = 
	
	// 업 카운트 ... ex) 1, 2, 3 ~ 65535
	TimHandle1.Init.CounterMode   = 
	
	// Input Capture Initialized
	if(HAL_TIM_IC_Init(&TimHandle1) != HAL_OK){ Error_Handler();}
	 
	// TIM_IC_InitTypeDef 구조체 선언
	sICConfig.ICPolarity  = 
	sICConfig.ICSelection = 
	sICConfig.ICPrescaler = 
	sICConfig.ICFilter    = 
	
	//uBrain의 초음파 3개는 각각 TIM3의 2,3,4 채널을 사용. 1번은 트리거
	HAL_TIM_IC_ConfigChannel(&TimHandle1, &sICConfig, TIM_CHANNEL_1);
	HAL_TIM_IC_ConfigChannel(&TimHandle1, &sICConfig, TIM_CHANNEL_2);
	HAL_TIM_IC_ConfigChannel(&TimHandle1, &sICConfig, TIM_CHANNEL_3);
	HAL_TIM_IC_ConfigChannel(&TimHandle1, &sICConfig, TIM_CHANNEL_4);

	HAL_TIM_IC_Start_IT(&TimHandle1, TIM_CHANNEL_2) ;
	HAL_TIM_IC_Start_IT(&TimHandle1, TIM_CHANNEL_3) ;
	HAL_TIM_IC_Start_IT(&TimHandle1, TIM_CHANNEL_4) ;


		
	/*============= ⑤ 그림 10. PWM Configuration 부분 ================*/
	
	// TIM10의 최대 클락 90Mhz (데이터시트 spec 참고) 계산식은 Reference Manual 참조
	// (SystemCoreClock / 131099) -1 = 1372 -> 90Mhz / 1372 = 65597Hz
	// 초당 2회 500m/s, 1회 65535(0.5초), 2회 131070(1초), 500 / 65535 = 7.5u/s
	uwPrescalerValue = (SystemCoreClock / 2 / 131099) - 1;
		
	// TIM_HandleTypeDef 구조체 선언	
	TimHandle2.Instance = 
	TimHandle2.Init.Prescaler     = 
	TimHandle2.Init.Period        = 
	TimHandle2.Init.ClockDivision = 
	TimHandle2.Init.CounterMode   = 
	HAL_TIM_PWM_Init(&TimHandle2);
	
	
	//TIM_OC_InitType 구초제 선언
	sConfig.OCMode     = TIM_OCMODE_PWM1;
	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig.OCFastMode = TIM_OCFAST_DISABLE;
	sConfig.Pulse = 2;  // 7.6u/s * 2
	HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_1);
	
	while (1)
	{
	} 
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */


/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  htim : hadc handle
  * @retval None
  */


/*============= ④ 그림 9. HAL_TIM_IC_CaptureCallback Function 부분 ================*/

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if((TIM3->CCER & TIM_CCER_CC2P) == 0)
			{
				/* Get the 1st Input Capture value */
				uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
				TIM3->CCER |= TIM_CCER_CC2P;
			}
			else if((TIM3->CCER & TIM_CCER_CC2P) == TIM_CCER_CC2P)
			{
				/* Get the 2nd Input Capture value */
				uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); 
				
				/* Capture computation */
				if (uwIC2Value2 > uwIC2Value1)
				{
						uwDiffCapture1 = (uwIC2Value2 - uwIC2Value1); 
				}
				else if (uwIC2Value2 < uwIC2Value1)
				{
						uwDiffCapture1 = ((0xFFFF - uwIC2Value1) + uwIC2Value2); 
				}
				else
				{
						uwDiffCapture1 = 0;
				}
				printf("\r\n Value Right : %d cm", );
					
				uwFrequency = (2*HAL_RCC_GetPCLK1Freq()) / uwDiffCapture1;
				TIM3->CCER &= ~TIM_CCER_CC2P;
			}
		}

		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			if((TIM3->CCER & TIM_CCER_CC3P) == 0)
			{
				/* Get the 1st Input Capture value */
				uwIC2Value3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
				TIM3->CCER |= TIM_CCER_CC3P;
			}
			else if((TIM3->CCER & TIM_CCER_CC3P) == TIM_CCER_CC3P)
			{
				/* Get the 2nd Input Capture value */
				uwIC2Value4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); 
				
				/* Capture computation */
				if (uwIC2Value4 > uwIC2Value3)
				{
					uwDiffCapture2 = (uwIC2Value4 - uwIC2Value3); 
				}
				else if (uwIC2Value4 < uwIC2Value3)
				{
					uwDiffCapture2 = ((0xFFFF - uwIC2Value3) + uwIC2Value4); 
				}
				else
				{
					uwDiffCapture2 = 0;
				}
				printf("\r\n Value Forward : %d cm", ); 

				uwFrequency = (2*HAL_RCC_GetPCLK1Freq()) / uwDiffCapture2;
				TIM3->CCER &= ~TIM_CCER_CC3P;
			}		
		}
		
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
			if((TIM3->CCER & TIM_CCER_CC4P) == 0)
			{
				/* Get the 1st Input Capture value */
				uwIC2Value5 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
				TIM3->CCER |= TIM_CCER_CC4P;
			}
			else if((TIM3->CCER & TIM_CCER_CC4P) == TIM_CCER_CC4P)
			{
				/* Get the 2nd Input Capture value */
				uwIC2Value6 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); 
				
				/* Capture computation */
				if (uwIC2Value6 > uwIC2Value5)
				{
					uwDiffCapture3 = (uwIC2Value6 - uwIC2Value5); 
				}
				else if (uwIC2Value6 < uwIC2Value5)
				{
					uwDiffCapture3 = ((0xFFFF - uwIC2Value5) + uwIC2Value6); 
				}
				else
				{
					uwDiffCapture3 = 0;
				}
				printf("\r\n Value Left: %d cm", );
					
				uwFrequency = (2*HAL_RCC_GetPCLK1Freq()) / uwDiffCapture3;
				TIM3->CCER &= ~TIM_CCER_CC4P;
			}
		}
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
    /* Turn LED3 on */
    BSP_LED_On(LED3);
    while(1)
    {
    }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 360;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig (&RCC_OscInitStruct);

	/* Activate the Over-Drive mode */
	HAL_PWREx_ActivateOverDrive();

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
	 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/**
  * @brief TIM MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param htim: TIM handle pointer
  * @retval None
  */

/*============= ⑥ 그림 11. HAL_TIM_PWM_MspInit Function 부분 ================*/

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if(htim->Instance == TIM10)	
	{
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* TIMx Peripheral clock enable */
		__TIM10_CLK_ENABLE();
		/* Enable GPIO Channels Clock */
		__GPIOF_CLK_ENABLE();
	 		
		/*##-2- Configure I/Os #############	########################################*/
				GPIO_InitStruct.Pin = GPIO_PIN_6 ;
		
		/* Common configuration for all channels */
		GPIO_InitStruct.Mode =
		GPIO_InitStruct.Pull =
		GPIO_InitStruct.Speed =
		GPIO_InitStruct.Alternate = 
		
		/* Channel 3 configuration */
		HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
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

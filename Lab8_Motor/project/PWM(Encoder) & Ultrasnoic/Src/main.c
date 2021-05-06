/**
  ******************************************************************************
  * @file    TIM/TIM_PWMOutput/Src/main.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    26-February-2014
  * @brief   This sample code shows how to use STM32F4xx TIM HAL API to generate
  *          4 signals in PWM.
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

/** @addtogroup TIM_PWM_Output
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
#define  PERIOD_VALUE       0xFFFF  /* Period Value  */
#define  PULSE1_VALUE       0xFFFF        /* Capture Compare 1 Value  */
#define  PULSE2_VALUE       900         /* Capture Compare 2 Value  */
#define  PULSE3_VALUE       600         /* Capture Compare 3 Value  */
#define  PULSE4_VALUE       450         /* Capture Compare 4 Value  */

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Timer handler declaration */
TIM_HandleTypeDef    TimHandle1, TimHandle2, TimHandle3, TimHandle4;
TIM_IC_InitTypeDef     sICConfig;
TIM_Encoder_InitTypeDef sENConfig;
	
/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig1, sConfig2, sConfig3;

/* Counter Prescaler value */
uint32_t uwPrescalerValue = 0;
uint16_t motorInterrupt1 = 0;
uint16_t motorInterrupt2 = 0;

uint8_t encoder_right = READY ;
uint8_t encoder_left  = READY ;
	 

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
void Motor_Forward(void);
void Motor_Backward(void);
void Motor_Left(void);
void Motor_Right(void);
void Motor_Stop(void);
void Motor_Speed_Up_Config(void);
void Motor_Speed_Down_Config(void);
static void EXTILine_Config(void);
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
	uint8_t ch;
	uint16_t Encoder1 = 0;
	uint16_t Encoder2 = 0;
	GPIO_InitTypeDef  GPIO_InitStruct;
	
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
	/* Compute the prescaler value to have TIM4,TIM8 counter clock equal to 2 MHz */
	uwPrescalerValue = (SystemCoreClock/2)/1000000;
	

	// PB2 모터 전원 인가를 위한 GPIO 초기화
	__GPIOB_CLK_ENABLE();
		
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // MC_EN(PB2) 모터 전원 
	
	
	/*##-1- Configure the TIM peripheral #######################################*/ 
	/* Initialize TIMx peripheral as follow:
	   + Prescaler = (SystemCoreClock/2)/18000000
	   + Period = 1800  (to have an output frequency equal to 10 KHz)
	   + ClockDivision = 0
	   + Counter direction = Up
	*/
	sConfig1.OCMode     = TIM_OCMODE_PWM1;
	sConfig1.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig1.OCFastMode = TIM_OCFAST_DISABLE;
	sConfig1.Pulse = 20000;
	
	// 왼쪽 Motor가 연결되어 있는 PIN은 회로도를 보면 PC6 / PC7 이며 Datasheet를 보면 PC6 / P7로 사용할 수 있는 Timer와 채널이 나와있다.
	TimHandle1.Instance = TIM8;	// MC_1A(PC6) -> TIM8_CH1, MC_2A(PC7) -> TIM8_CH2
	TimHandle1.Init.Prescaler     = uwPrescalerValue;
	TimHandle1.Init.Period        = 20000; 
	TimHandle1.Init.ClockDivision = 0;
	TimHandle1.Init.CounterMode   = TIM_COUNTERMODE_UP;
	HAL_TIM_PWM_Init(&TimHandle1);
	
	HAL_TIM_PWM_ConfigChannel(&TimHandle1, &sConfig1, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&TimHandle1, &sConfig1, TIM_CHANNEL_2);
		
	/*## Configure the PWM channels #########################################*/ 
	/* Common configuration for all channels */
	sConfig2.OCMode     = TIM_OCMODE_PWM1;
	sConfig2.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig2.OCFastMode = TIM_OCFAST_DISABLE;
	sConfig2.Pulse = 20000;
	
	TimHandle2.Instance = TIM4;	// MC_1B(PD12) -> TIM4_CH1, MC_2B(PD13) -> TIM4_CH2
	TimHandle2.Init.Prescaler     = uwPrescalerValue;
	TimHandle2.Init.Period        = 20000;
	TimHandle2.Init.ClockDivision = 0;
	TimHandle2.Init.CounterMode   = TIM_COUNTERMODE_UP;
	HAL_TIM_PWM_Init(&TimHandle2);

	HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig2, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig2, TIM_CHANNEL_2);

	EXTILine_Config(); // Encoder Interrupt Setting
	
	 //  --------------------------------------- Encoder ----------------------------------------------------START
	/*
	sENConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sENConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sENConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sENConfig.IC1Polarity =TIM_ICPOLARITY_RISING ;
	sENConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sENConfig.IC1Filter    = 0;  	
	sENConfig.IC2Filter    = 0;
	sENConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sENConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	
	TimHandle3.Instance = TIM5;
	
	TimHandle3.Init.Period        = 1000;
	TimHandle3.Init.Prescaler     = uwPrescalerValue;
	TimHandle3.Init.ClockDivision = 0;
	TimHandle3.Init.CounterMode   = TIM_COUNTERMODE_UP;
	
	HAL_TIM_Encoder_Init(&TimHandle3, &sENConfig);

	TimHandle4.Instance = TIM2;
	
	TimHandle4.Init.Period        = 1000;
	TimHandle4.Init.Prescaler     = uwPrescalerValue;
	TimHandle4.Init.ClockDivision = 0;
	TimHandle4.Init.CounterMode   = TIM_COUNTERMODE_UP;
	
	HAL_TIM_Encoder_Init(&TimHandle4, &sENConfig);
	
	HAL_TIM_Encoder_Start(&TimHandle3,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&TimHandle3,TIM_CHANNEL_2);

	HAL_TIM_Encoder_Start(&TimHandle4,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&TimHandle4,TIM_CHANNEL_2);
	*/
	//  -------------------------------------- Encoder ----------------------------------------------------END
	
	printf("\r\n ------------------- Encoder Motor Example -------------------");
	printf("\r\n'w' = Forward, 's' = Stop, 'x' = Backward, 'a' = Left, 'd' = Right ");
	printf("\r\n'1' = Speed Up, '2' = Speed Down");
	
	
	while (1)
	{
		ch=BSP_UART_GetChar(COM1);

		// Encoder 값이 변경되었을 때만 업데이트 한다. 
		/* Encoder 값이 1300 변화 당 1 바퀴 이동 */
		/* 13pulse * 감속비 100  = 1300 pulse */
		if(Encoder1 != motorInterrupt1 || Encoder2 != motorInterrupt2)
		{
			printf("\r\nEncoder 1(Left) -> %d", motorInterrupt1);
			printf("\r\nEncoder 2(Right) -> %d",  motorInterrupt2);
			Encoder1 = motorInterrupt1;
			Encoder2 = motorInterrupt2;
			HAL_Delay(50);
		}
		switch(ch)
		{
			case 'w' : // 'w' 인지 확인
				Motor_Stop();
				Motor_Forward();
				break;
			
			case 'x' : // 'x' 인지 확인
				Motor_Stop();
				Motor_Backward();
				break;
			
			case 'a' : // 'a' 인지 확인
				Motor_Stop();
				Motor_Left();
				break;
			
			case 'd' : // 'd' 인지 확인
				Motor_Stop();
				Motor_Right();
				break;
			
			case 's' : // 's' 인지 확인
			  Motor_Stop();
				break;
			
			case '1' :   // Speed Up 
				if(sConfig1.Pulse <20000)
				{
					Motor_Speed_Up_Config();
				}
				break;

			case '2' : // Speed Down
				if(sConfig1.Pulse > 0)
				{
					Motor_Speed_Down_Config();
				}
				break;
		}
	}
}

void Motor_Forward(void)
{
	HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_2);
}

void Motor_Backward(void)
{
	HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_1);
}
void Motor_Left(void)
{
	HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_2);
}
void Motor_Right(void)
{
	HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_1);
}

void Motor_Stop(void)
{
	HAL_TIM_PWM_Stop(&TimHandle1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&TimHandle1, TIM_CHANNEL_2);				
	HAL_TIM_PWM_Stop(&TimHandle2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&TimHandle2, TIM_CHANNEL_2);
}

void Motor_Speed_Up_Config(void)
{
	sConfig1.Pulse  = sConfig1.Pulse + 100;
	sConfig2.Pulse  = sConfig2.Pulse + 100;
	TIM8->CCR1 = sConfig1.Pulse;
	TIM8->CCR2 = sConfig1.Pulse;
	TIM4->CCR1 = sConfig2.Pulse;
	TIM4->CCR2 = sConfig2.Pulse;
}

void Motor_Speed_Down_Config(void)
{
	sConfig1.Pulse  = sConfig1.Pulse - 100;
	sConfig2.Pulse  = sConfig2.Pulse - 100;
	TIM8->CCR1 = sConfig1.Pulse;
	TIM8->CCR2 = sConfig1.Pulse;
	TIM4->CCR1 = sConfig2.Pulse;
	TIM4->CCR2 = sConfig2.Pulse;
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
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

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
  * @brief  Configures EXTI Line (connected to PA15, PB3, PB4, PB5 pin) in interrupt mode
  * @param  None
  * @retval None
  */
static void EXTILine_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOA clock */
  __GPIOA_CLK_ENABLE();
  
  /* Configure PA15 pin  */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_15 ;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
/* Enable and set EXTI Line15 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
		
	
 /* Enable GPIOB clock */	
 __GPIOB_CLK_ENABLE();

  /* Configure PB3, PB4, PB5 pin  */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
  /* Enable and set EXTI Line4 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

/**
  * @}
  */
/******************** ④ [Example/User/main.c - 467 line] **************************/
/****************** 여기에 Encoder 값을 Callback해주는 함수를 작성 *******************/
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {

	 
	 
	 
	 
	 
	 
	 
	 
	 

 }
/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

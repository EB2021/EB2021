// LED 예제

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

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
  * @brief  Main program

      This is the main of Lab 5. UART

  * @param  None
  * @retval None

      Template by HSK

Description : 

			 2015. 4. 20
  */
int main(void)
{    
	uint8_t ch;
		
	/* STM32F4xx HAL library initialization:
	   - Configure the Flash prefetch, instruction and Data caches
	   - Configure the Systick to generate an interrupt each 1 msec
	   - Set NVIC Group Priority to 4
	   - Global MSP (MCU Support Package) initialization
	 */
	HAL_Init();

	/* Configure the system clock to 180 Mhz */
	SystemClock_Config();

	/* Configure LED1, LED2 and LED3 */
	BSP_LED_Init(LED1); // LED1 초기화 하기
	BSP_LED_Init(LED2); // LED2 초기화 하기
	BSP_LED_Init(LED3); // LED3 초기화 하기
	BSP_LED_Init(LED4); // LED4 초기화 하기

	BSP_COM1_Init(); // USART3 초기화 하기
	printf("\r\n LED Toggle Test - '1'(LED1), '2'(LED2), '3'(LED3), '4'(LED4) : ");

	/* Infinite loop */  
	while (1)
	{
		ch = BSP_UART_GetChar(COM1); // USART3에서 1바이트 가져오기
		
		if(ch) // USART3에서 유효한 데이터를 가져왔을 경우
		{
			switch(ch)
			{
				case '1': // '1' 인지 확인
					putchar(ch); // 받은 데이터 USART3으로 출력하기
					BSP_LED_Toggle(LED1); // LED1 Toggle 하기
					printf("\r\n-> LED1 Toggle ");
					break;
				case '2': // '2' 인지 확인
					putchar(ch); // 받은 데이터 USART3으로 출력하기
					BSP_LED_Toggle(LED2); // LED2 Toggle 하기
					printf("\r\n-> LED2 Toggle ");
					break;
				case '3': // '3' 인지 확인
					putchar(ch); // 받은 데이터 USART3으로 출력하기
					BSP_LED_Toggle(LED3); // LED3 Toggle 하기
					printf("\r\n-> LED3 Toggle ");
					break;
				case '4': // '4' 인지 확인
					putchar(ch); // 받은 데이터 USART3으로 출력하기
					BSP_LED_Toggle(LED4); // LED4 Toggle 하기
					printf("\r\n-> LED4 Toggle ");
					break;
			}
			printf("\r\n LED Toggle Test - '1'(LED1), '2'(LED2), '3'(LED3), '4'(LED4) : ");
		}
	}
}

}



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
	/*
			f(VCO clock) = f(PLL clock input) × (PLLN / PLLM)
			f(PLL general clock output) = f(VCO clock) / PLLP
			f(USB OTG FS, SDIO, RNG clock output) = f(VCO clock) / PLLQ
      
	    f(VCO clock)	 360 = 25 * (360/25)
      f(PLL general clock output)   180 = 360  /2 	
	    f(USB OTG FS, SDIO, RNG clock output) 51.4 =  360 /7
	*/
	
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 360;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

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
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	/* Turn LED3 on: Transfer error in reception/transmission process */
	BSP_LED_On(LED3); 
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




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

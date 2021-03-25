#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"


typedef enum 
{
  LED1 = 0,
  LED2 = 1,
  LED3 = 2,
  LED4 = 3

}Led_TypeDef;



																 
#define LEDn                             4

#define LED1_PIN                                	GPIO_PIN_8
#define LED1_GPIO_PORT                  	GPIOI
#define LED1_GPIO_CLK_ENABLE()      __GPIOI_CLK_ENABLE()  
#define LED1_GPIO_CLK_DISABLE()     __GPIOI_CLK_DISABLE()  
    
    
#define LED2_PIN                         				GPIO_PIN_9
#define LED2_GPIO_PORT                   GPIOI
#define LED2_GPIO_CLK_ENABLE()      __GPIOI_CLK_ENABLE()   
#define LED2_GPIO_CLK_DISABLE()     __GPIOI_CLK_DISABLE()  
  
#define LED3_PIN                         				GPIO_PIN_10
#define LED3_GPIO_PORT                   GPIOI
#define LED3_GPIO_CLK_ENABLE()      __GPIOI_CLK_ENABLE()   
#define LED3_GPIO_CLK_DISABLE()     __GPIOI_CLK_DISABLE()  
  
#define LED4_PIN                         				GPIO_PIN_11
#define LED4_GPIO_PORT                   GPIOI
#define LED4_GPIO_CLK_ENABLE()      __GPIOI_CLK_ENABLE()   
#define LED4_GPIO_CLK_DISABLE()     __GPIOI_CLK_DISABLE()  
    
#define LEDx_GPIO_CLK_ENABLE(__INDEX__)   (((__INDEX__) == 0) ? LED1_GPIO_CLK_ENABLE() :\
                                           ((__INDEX__) == 1) ? LED2_GPIO_CLK_ENABLE() :\
                                           ((__INDEX__) == 2) ? LED3_GPIO_CLK_ENABLE() : LED4_GPIO_CLK_ENABLE())

#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  (((__INDEX__) == 0) ? LED1_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 1) ? LED2_GPIO_CLK_DISABLE() :\
                                           ((__INDEX__) == 2) ? LED3_GPIO_CLK_DISABLE() : LED4_GPIO_CLK_DISABLE())



GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT,
                                 LED2_GPIO_PORT,
                                 LED3_GPIO_PORT,
                                 LED4_GPIO_PORT};

const uint16_t GPIO_PIN[LEDn] = {LED1_PIN,
                                 LED2_PIN,
                                 LED3_PIN,
                                 LED4_PIN};


static void SystemClock_Config(void);
static void BSP_LED_Init(Led_TypeDef Led);
static void BSP_LED_On(Led_TypeDef Led);
static void BSP_LED_Off(Led_TypeDef Led);
static void BSP_LED_Toggle(Led_TypeDef Led);


int main(void)
{

	  volatile unsigned int i;
	
	/* STM32F4xx HAL library initialization:
	   - Configure the Flash prefetch, instruction and Data caches
	   - Configure the Systick to generate an interrupt each 1 msec
	   - Set NVIC Group Priority to 4
	   - Global MSP (MCU Support Package) initialization
	 */
	
	HAL_Init(); //HAL(Hardware Abstraction Layer)은 하드웨어와 소프트웨어 사이의 추상화 계층
	
	
	/* Configure the system clock to 180 Mhz */
	SystemClock_Config();
	
	/* Configure LED1, LED2 and LED3 */
	BSP_LED_Init(LED1); // LED1 초기화 하기
	BSP_LED_Init(LED2); // LED2 초기화 하기
	BSP_LED_Init(LED3); // LED3 초기화 하기
	BSP_LED_Init(LED4); // LED4 초기화 하기
	
	
	
	while (1)
	{
        	BSP_LED_On(LED1); // LED1 Toggle 하기
		
          for(i=0; i < 999999; i++);				
		
					BSP_LED_On(LED2); // LED2 Toggle 하기
		
		      for(i=0; i < 999999; i++);		
		
					BSP_LED_On(LED3); // LED3 Toggle 하기
		
          for(i=0; i < 999999; i++);							
		
		      BSP_LED_On(LED4); // LED4 Toggle 하기
		
		      for(i=0; i < 999999; i++);		
		
		
		
        	BSP_LED_Off(LED1); // LED1 Toggle 하기
		
          for(i=0; i < 999999; i++);				
					
					BSP_LED_Off(LED2); // LED2 Toggle 하기
					
		      for(i=0; i < 999999; i++);		
					
					BSP_LED_Off(LED3); // LED3 Toggle 하기
					
          for(i=0; i < 999999; i++);							
					
		      BSP_LED_Off(LED4); // LED4 Toggle 하기
					
		      for(i=0; i < 999999; i++);				
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
  * @brief  Configures LED GPIO.
  * @param  Led: LED to be configured. 
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  * @retval None
  */

void BSP_LED_Init(Led_TypeDef Led)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	/* Enable the GPIO_LED clock */
	LEDx_GPIO_CLK_ENABLE(Led);

	/* Configure the GPIO_LED pin */

	
	 /***************************************************

       실습 3. GPIO 프로그래밍 실습 

   /***************************************************
	
	/*
	GPIO_InitStruct.Pin = 
	GPIO_InitStruct.Mode = 
	GPIO_InitStruct.Pull =
	GPIO_InitStruct.Speed =
	*/

	
	GPIO_InitStruct.Pin = GPIO_PIN[Led];
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	
	HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: LED to be set on 
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  * @retval None
  */
void BSP_LED_On(Led_TypeDef Led)
{
HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}

/**
  * @brief  Turns selected LED Off. 
  * @param  Led: LED to be set off
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  * @retval None
  */
void BSP_LED_Off(Led_TypeDef Led)
{
HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET); 
}


/**
  * @brief  Toggles the selected LED.
  * @param  Led: LED to be toggled
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  * @retval None
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
	HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}


/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

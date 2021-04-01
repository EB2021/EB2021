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

#define STK_CTRL_ADDRESS											((volatile uint32_t*)0xE000E010 )
#define STK_LOAD_ADDRESS											((volatile uint32_t*)0xE000E014 )
#define STK_VAL_ADDRESS											((volatile uint32_t*)0xE000E018 )



GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT,
                                 LED2_GPIO_PORT,
                                 LED3_GPIO_PORT,
                                 LED4_GPIO_PORT};

const uint16_t GPIO_PIN[LEDn] = {LED1_PIN,
                                 LED2_PIN,
                                 LED3_PIN,
                                 LED4_PIN};


static void BSP_LED_Init(Led_TypeDef Led);
static void BSP_LED_On(Led_TypeDef Led);
static void BSP_LED_Off(Led_TypeDef Led);
static void BSP_LED_Toggle(Led_TypeDef Led);

unsigned int cnt = 0;
																 
int main(void)
{

	  volatile unsigned int i;
	/*****************************************/
	/*	인터럽트를 사용하기 위한 설정을 작성하시오. */
	/*****************************************/
	
	*(STK_CTRL_ADDRESS) = 
	/* use processor clock
		assert the SysTick exception request
		counter enable */
	
	*(STK_LOAD_ADDRESS) = 
	/*set value of systemcoreclock - 1*/
	
	*(STK_VAL_ADDRESS) = 
	/*set 0*/
	
	
	
	/*****************************************/
	
	
	/* Configure LED1, LED2 and LED3 */
	BSP_LED_Init(LED1); // LED1 초기화 하기
	BSP_LED_Init(LED2); // LED2 초기화 하기
	BSP_LED_Init(LED3); // LED3 초기화 하기
	BSP_LED_Init(LED4); // LED4 초기화 하기
	
	
	
	while (1)
	{
		
        	
					BSP_LED_On(LED2); // LED2 Toggle 하기
		      for(i=0; i < 999999; i++);		
					BSP_LED_On(LED3); // LED3 Toggle 하기
          for(i=0; i < 999999; i++);							
		      BSP_LED_On(LED4); // LED4 Toggle 하기
		      for(i=0; i < 999999; i++);		
		
      
					BSP_LED_Off(LED2); // LED2 Toggle 하기
		      for(i=0; i < 999999; i++);		
					BSP_LED_Off(LED3); // LED3 Toggle 하기
          for(i=0; i < 999999; i++);							
		      BSP_LED_Off(LED4); // LED4 Toggle 하기
		      for(i=0; i < 999999; i++);				
	   
	}
}

/*******************************************/
/*	이곳에 SysTick 인터럽트 핸들러를 작성하시오. */
/*******************************************/

void SysTick_Handler(void){
					
}



/***
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


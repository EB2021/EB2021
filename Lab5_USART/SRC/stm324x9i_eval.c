/**
  ******************************************************************************
  * @file    stm324x9i_eval.c
  * @author  MCD Application Team
  * @version V2.0.1
  * @date    26-February-2014
  * @brief   This file provides a set of firmware functions to manage LEDs, 
  *          push-buttons and COM ports available on STM324x9I-EVAL evaluation 
  *          board(MB1045) RevB from STMicroelectronics.
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
  
/* File Info: ------------------------------------------------------------------
                                   User NOTE

   This driver requires the stm324x9i_eval_io to manage the joystick

------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm324x9i_eval.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>



GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT,
                                 LED2_GPIO_PORT,
                                 LED3_GPIO_PORT,
                                 LED4_GPIO_PORT};

const uint16_t GPIO_PIN[LEDn] = {LED1_PIN,
                                 LED2_PIN,
                                 LED3_PIN,
                                 LED4_PIN};

/* UART handler declaration */
UART_HandleTypeDef UartHandle1, UartHandle2;
USART_TypeDef* COM_USART[COMn] = {EVAL_COM1, EVAL_COM2};
GPIO_TypeDef* COM_TX_PORT[COMn] = {EVAL_COM1_TX_GPIO_PORT, EVAL_COM2_TX_GPIO_PORT};
GPIO_TypeDef* COM_RX_PORT[COMn] = {EVAL_COM1_RX_GPIO_PORT, EVAL_COM2_RX_GPIO_PORT};
const uint16_t COM_TX_PIN[COMn] = {EVAL_COM1_TX_PIN, EVAL_COM2_TX_PIN};
const uint16_t COM_RX_PIN[COMn] = {EVAL_COM1_RX_PIN, EVAL_COM2_RX_PIN};
const uint16_t COM_TX_AF[COMn] = {EVAL_COM1_TX_AF, EVAL_COM2_TX_AF};
const uint16_t COM_RX_AF[COMn] = {EVAL_COM1_RX_AF, EVAL_COM2_RX_AF};
uint8_t GetChBuf[2];






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
  * @brief  Configures COM port.
  * @param  COM: COM port to be configured.
  *          This parameter can be one of the following values:
  *            @arg  COM1 
  *            @arg  COM2
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains the
  *                configuration information for the specified USART peripheral.
  * @retval None
  */
void BSP_COM_Init(COM_TypeDef COM, UART_HandleTypeDef *huart)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	
	/* Enable GPIO clock */
	EVAL_COMx_TX_GPIO_CLK_ENABLE(COM);
	EVAL_COMx_RX_GPIO_CLK_ENABLE(COM);

	/* Enable USART clock */
	EVAL_COMx_CLK_ENABLE(COM);

	
	/* Configure GPIO USART Tx as alternate function */
	/*  (1) 이 부분에 USART 3 의 TX를 alternate function으로 설정하는 GPIO 작업을 작성할 것 */
	/*
					Parameter 설정 : 
						Alternate function, 
						Push-Pull mode
						Speed Fast, 
						PullUp
				Pin과 Alternate는  COM_TX_PIN[COM], COM_TX_AF[COM] 배열 참조
	*//*
	GPIO_InitStruct.Pin = 
	GPIO_InitStruct.Mode = 
	GPIO_InitStruct.Speed = 
	GPIO_InitStruct.Pull = 
	GPIO_InitStruct.Alternate = 
	HAL_GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStruct);*/
																							
	/* Configure GPIO USART Rx as alternate function */
	/*  (2)  이 부분에 USART 3 의 RX를 alternate function으로 설정하는 GPIO 작업을 작성할 것 */
	/*
					Parameter 설정 : 
						Alternate function, 
						Push-Pull mode
					Pin과 Alternate는  COM_RX_PIN[COM], COM_RX_AF[COM] 배열 참조
	*//*
	GPIO_InitStruct.Pin = 
	GPIO_InitStruct.Mode = 
	GPIO_InitStruct.Alternate = 
	HAL_GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStruct);*/
		 
	/* NVIC for USART */
	HAL_NVIC_SetPriority(EVAL_COMx_INT_ENABLE(COM), 5, 0);
	HAL_NVIC_EnableIRQ(EVAL_COMx_INT_ENABLE(COM));

	/* USART configuration */
	huart->Instance = COM_USART[COM];
	HAL_UART_Init(huart);
		
		
}

void BSP_COM1_Init(void)
{
	/* Configure the USART3 peripheral ######################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART3 configured as follow:
	  - Word Length = 8 Bits
	  - Stop Bit = One Stop bit
	  - Parity = NONE parity
	  - BaudRate = 115200 baud
	  - Hardware flow control disabled (RTS and CTS signals) */
	UartHandle1.Instance = EVAL_COM1;

	/* USART Register configuration */
	/*
	     (3) 이 부분에 시리얼 포트를 초기화하는 코드를 작성하라
			 stm32f4xx_hal_uart.h 참조
	*/
	/*
	UartHandle1.Init.BaudRate = 
	UartHandle1.Init.WordLength = 
	UartHandle1.Init.StopBits = 
	UartHandle1.Init.Parity = 
	UartHandle1.Init.HwFlowCtl = 
	UartHandle1.Init.Mode = UART_MODE_TX | UART_MODE_RX;*/
	
	BSP_COM_Init(COM1, &UartHandle1);
}

void BSP_COM2_Init(void)
{
	/* Configure the USART peripheral ######################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART6 configured as follow:
	  - Word Length = 8 Bits
	  - Stop Bit = One Stop bit
	  - Parity = NONE parity
	  - BaudRate = 115200 baud
	  - Hardware flow control disabled (RTS and CTS signals) */
	UartHandle2.Instance = EVAL_COM2;

	UartHandle2.Init.BaudRate = 115200;
	UartHandle2.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle2.Init.StopBits = UART_STOPBITS_1;
	UartHandle2.Init.Parity = UART_PARITY_NONE;
	UartHandle2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartHandle2.Init.Mode = UART_MODE_TX | UART_MODE_RX;
	
	BSP_COM_Init(COM2, &UartHandle2);
}

/*******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/


uint8_t BSP_UART_GetChar(uint8_t COM)
{
	uint8_t ch = 0;
	
	if(COM == COM1)
	{
		if( (UartHandle1.Instance->SR & UART_FLAG_RXNE) == UART_FLAG_RXNE)
		{
			ch = UartHandle1.Instance->DR & 0xFF;
		}
	}
	else if(COM == COM2)
	{
		if( (UartHandle2.Instance->SR & UART_FLAG_RXNE) == UART_FLAG_RXNE)
		{
			ch = UartHandle2.Instance->DR & 0xFF;
		}
	}
	
	return ch;
}

void BSP_UART_PutChar(uint8_t COM, uint8_t ch)
{
	if(COM == COM1)
	{
		HAL_UART_Transmit(&UartHandle1, &ch, 1, 10);
	}
	else if(COM == COM2)
	{
		HAL_UART_Transmit(&UartHandle2, &ch, 1, 10);
	}
}

void BSP_UART_Printf(uint8_t COM, const char *fmt,...)
{
	va_list ap;
	char string[1024];
	unsigned int length, i;

	va_start(ap, fmt);
	vsprintf(string, fmt, ap);
	length = strlen(string);
	for(i=0; i<length; i++)
	{
		if(string[i] == '\n') putchar('\r');
		BSP_UART_PutChar(COM, string[i]);
	}
	va_end(ap);
}

    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

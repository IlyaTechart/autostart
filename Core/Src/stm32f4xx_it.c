/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "stm32f411xe.h"
#include "KEY.h"
#include "EPD_Test.h"
#include "EPD_1in54_V2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

extern UART_HandleTypeDef huart1;
extern uint8_t Recive_array[2];
extern KEY_Typedef_flags* event_flags;
bool state = 1;
extern uint8_t* BlackImage;
extern uint32_t refresh_e_ink_cnt;
extern bool falg_disp_sleep;

uint32_t start_time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */


  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	refresh_e_ink_cnt++;

	if( (refresh_e_ink_cnt > 30) && !falg_disp_sleep)
	{
//	    EPD_1IN54_V2_Init();
		EPD_1IN54_V2_ReadBusy();
	    EPD_1IN54_V2_Init();
	    EPD_1IN54_V2_Clear();
	    EPD_1IN54_V2_Sleep();
	    DEV_Module_Exit();
	    refresh_e_ink_cnt = 0;
	    falg_disp_sleep = 1;
	}

	if(event_flags->k_15[1])
	{
		uint8_t string_[4];
		string_[3] = '\0';
		start_time++;
		sprintf((char*)string_,"%d",start_time);
	    Paint_ClearWindows(1, 111, 1 + Font16.Width * 3, 111 + Font16.Height, WHITE);
	    Paint_DrawString_EN(1, 111, (char*)string_, &Font16, WHITE, BLACK);
	    if(!falg_disp_sleep)
	    {
		    EPD_1IN54_V2_DisplayPart(BlackImage);
	    }
	}else{
	    Paint_ClearWindows(1, 111, 1 + Font16.Width * 3, 111 + Font16.Height, WHITE);
	    EPD_1IN54_V2_DisplayPart(BlackImage);
		start_time = 0;
	}

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  TIM2->SR &= ~TIM_SR_UIF;

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	static uint32_t cnt = 0;
	uint8_t data1 = 0;

	static bool SByte1 = 0;
	static bool SByte2 = 0;
	static bool SByte3 = 0;
	static bool enter = 0;

    if(enter)
    {
    	while(cnt < 6)
    	{
    		while((USART1->SR & USART_SR_RXNE) != USART_SR_RXNE){}
    		Recive_array[cnt] = (uint8_t)(USART1->DR & 0xFF);
    		cnt++;
    	}

    	SByte1 = 0;
    	SByte2 = 0;
    	SByte3 = 0;
    	enter = 0;
    	cnt = 0;
    	state = 0;
    }

    if (USART1->SR & USART_SR_RXNE) {
    	data1 = (uint8_t)(USART1->DR & 0xFF);  // Чтение данных из регистра
    }

    if(data1 == 0xAA)
    {
    	SByte1 = 1;
    }else if(data1 == 0xBB)
    {
    	SByte2 = 1;
    }else if(data1 == 0xCC)
    {
    	SByte3 = 1;
    	enter = 1;
    }


  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */


void EXTI9_5_IRQHandler(void)
{
	NVIC_DisableIRQ(EXTI15_10_IRQn);
	HAL_Delay(250);
	if( (GPIOB->IDR & GPIO_IDR_ID5) != GPIO_IDR_ID5)
	{
		KEY_Press_Button(NUM_BUTTON_2);
	}
	if(falg_disp_sleep)
	{
		EXIT_Eink_of_sleep();
	}
	refresh_e_ink_cnt = 0;
	HAL_Delay(500);
	EXTI->PR |= EXTI_PR_PR5;
	NVIC_EnableIRQ(EXTI15_10_IRQn);

}

void EXTI15_10_IRQHandler(void)
{
	NVIC_DisableIRQ(EXTI9_5_IRQn);
	HAL_Delay(250);
	if( (GPIOA->IDR & GPIO_IDR_ID15) != GPIO_IDR_ID15)
	{
		KEY_Press_Button(NUM_BUTTON_1);
	}
	if(falg_disp_sleep)
	{
		EXIT_Eink_of_sleep();
	}
	refresh_e_ink_cnt = 0;
	HAL_Delay(500);
	EXTI->PR |= EXTI_PR_PR15;
	NVIC_EnableIRQ(EXTI9_5_IRQn);

}

void EXIT_Eink_of_sleep(void)
{
	DEV_Module_Init();
	EPD_1IN54_V2_Init();
	EPD_1IN54_V2_DisplayPartBaseImage(BlackImage);
    EPD_1IN54_V2_Init_Partial();
    Paint_SelectImage(BlackImage);
	refresh_e_ink_cnt = 0;
	falg_disp_sleep = 0;
}

void EXTI0_IRQHandler(void)
{
	HAL_Delay(250);
	if( (GPIOA->IDR & GPIO_IDR_ID0) != GPIO_IDR_ID0)
	{
		KEY_Request_state();
	}
	if(falg_disp_sleep)
	{
		EXIT_Eink_of_sleep();
	}
	refresh_e_ink_cnt = 0;
	HAL_Delay(500);

	EXTI->PR |= EXTI_PR_PR0;
}

/* USER CODE END 1 */

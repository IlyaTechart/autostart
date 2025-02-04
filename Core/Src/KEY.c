/*
 * KEY.c
 *
 *  Created on: Nov 23, 2024
 *      Author: I
 */

#include "KEY.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>  // для malloc
#include "stm32f4xx_hal.h"
#include "stm32f411xe.h"
#include "EPD_Test.h"
#include "EPD_1in54_V2.h"



extern uint8_t* BlackImage;
uint32_t refresh_e_ink_cnt = 0;
bool falg_disp_sleep = 0;
uint8_t start_uart[3] = {0xCC, 0xBB, 0xAA};
uint8_t Recive_array[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
bool k_15 = 0; // klemma 15
const uint8_t TransmitAcepted[2] = {0xaa, 0xaa};
const uint8_t Error[2] = {0xff, 0xff};

KEY_Typedef_flags* event_flags;
KEY_Typedef_command* command;


static void KEY_Send_Command(uint8_t type_command,uint8_t value_command);
static void KEY_Error_Handler(void);
static void KEY_Flag_State_Handler(uint8_t id, uint8_t  comm);
static void KEY_Data_Handler(uint8_t id, uint8_t* data);
//static void KEY_Send_EngineStop(void);

extern UART_HandleTypeDef huart1;

void KEY_Init(void)
{
	event_flags = malloc(sizeof(KEY_Typedef_flags));

    event_flags->TransmitAcepted[0] = 0x20;
    event_flags->TransmitAcepted[1] = 0x20;
    event_flags->k_15[0] = 0x25;
    event_flags->k_15[1] = 0x00;
    event_flags->k_50[0] = 0x26;
    event_flags->k_50[1] = 0x00;
    event_flags->Error[0] = 0xFF;
    event_flags->Error[1] = 0xFF;

    command = malloc(sizeof(KEY_Typedef_command));

    command->struct_command.start_engine[0] = 0x00;
    command->struct_command.start_engine[1] = 0x00;
    command->struct_command.stop_engine[0] = 0x01;
    command->struct_command.stop_engine[1] = 0x00;

}

void KEY_Press_Button(uint8_t num_button)
{
	uint8_t case_ = 0;
	/*
	 * При case_ == 0 - мы удерживаем кнопку  меньше 1 сек
	 * При case == 1 - мы удерживаем кнопку больше 2 сек
	 */

	if(num_button == NUM_BUTTON_1)
	{
		case_ = 0;
	}else if(num_button == NUM_BUTTON_2)
	{
		case_ = 1;
	}else{
		GPIOC->ODR &= ~GPIO_PIN_13;
		HAL_Delay(50);
		GPIOC->ODR |= GPIO_PIN_13;
		HAL_Delay(50);
		GPIOC->ODR &= ~GPIO_PIN_13;
		HAL_Delay(50);
		GPIOC->ODR |= GPIO_PIN_13;
		HAL_Delay(50);
	}

	switch(case_)
	{

	case 0:  KEY_Send_Command(ID_START_ENGINE, COMMAND_START_ENGINE); break;

	case 1:  KEY_Send_Command(ID_STOP_ENGINE, COMMAND_STOP_ENGINE);  break;

	defult: Error_Handler();

	}
}

static void KEY_Send_Command(uint8_t type_command,uint8_t value_command)
{
	uint8_t tabel_comm[2][2] = {
	    {0, 1},
	    {1, 1}
	}; // arr[M][N] M — число строк (rows) N — число столбцов (columns).

//	*(command->ptr + (type_command * 2) + 1) = value_command;

	UART_Transmit(USART1, (uint8_t*)&tabel_comm[type_command][0], 2);
}

void KEY_ReciveData(void)
{
	UART_Receive(USART1, Recive_array, 2);
}

void KEY_Request_state(void)
{
	uint8_t command[2] = {0x03, 0x01};

	UART_Transmit(USART1, command, 2);
}

void KEY_Event(uint8_t* res_massage)
{
	uint8_t id_res_m = *(res_massage + 0);
	uint8_t com_res_m = *(res_massage + 1);

	if(id_res_m <= 0x19)
	{


	}else if( (0x20 <= id_res_m) && (id_res_m < 0x40))
	{
		KEY_Flag_State_Handler(id_res_m , com_res_m);

	}else if((id_res_m > 0x39) && (id_res_m < 0xAA))
	{
		KEY_Data_Handler(id_res_m, res_massage);
	}else
	{
		KEY_Error_Handler();
	}



//    Recive_array[0] = 0x0;
//    Recive_array[1] = 0x0;
}
static void KEY_Error_Handler(void)
{

	 Paint_ClearWindows(1, 130, 1 + Font16.Width * 5, 130 + Font16.Height, WHITE);
	 Paint_DrawString_EN(1, 130,"ERROR", &Font16, WHITE, BLACK);
	 EPD_1IN54_V2_DisplayPart(BlackImage);
	 HAL_Delay(500);
	 Paint_ClearWindows(1, 130, 1 + Font16.Width * 5, 130 + Font16.Height, WHITE);
	 Paint_DrawString_EN(1, 130,"ERROR", &Font16, WHITE, BLACK);
	 EPD_1IN54_V2_DisplayPart(BlackImage);
	 HAL_Delay(500);
	 Paint_ClearWindows(1, 130, 1 + Font16.Width * 5, 130 + Font16.Height, WHITE);
	 EPD_1IN54_V2_DisplayPart(BlackImage);
	 event_flags->Error[1] = 0x01;

}

static void KEY_Flag_State_Handler(uint8_t id, uint8_t comm)
{
	switch(id)
	{

//	case 0x20:  if(comm == 1)
//	{
//	 event_flags->Error[1] = 0x00;
//	 event_flags->TransmitAcepted[1] = 0x01;
//	 SH1106_DrawFilledCircle(100, 4, 4, 1);
//	 SH1106_UpdateScreen();
//	 HAL_Delay(50);
//	 SH1106_DrawFilledCircle(100, 4, 4, 0);
//	 SH1106_UpdateScreen();
//	 HAL_Delay(50);
//	 SH1106_DrawFilledCircle(100, 4, 4, 1);
//	 SH1106_UpdateScreen();
//	 HAL_Delay(50);
//	 SH1106_DrawFilledCircle(100, 4, 4, 0);
//	 SH1106_UpdateScreen();
//	 HAL_Delay(50);
//	 event_flags->TransmitAcepted[1] = 0x00;
//	}
//	break;

	case 0x25:  if(comm == 1)
	{
	 event_flags->Error[1] = 0x00;
	 event_flags->k_15[1] = 0x01;
	 Paint_ClearWindows(1, 1, 1 + Font16.Width * 5, 1 + Font16.Height, WHITE);
	 Paint_DrawString_EN(1, 1,"START", &Font16, BLACK, WHITE);
	 EPD_1IN54_V2_DisplayPart(BlackImage);
	}else if(comm == 0)
	{
	 event_flags->Error[1] = 0x00;
	 event_flags->k_15[1] = 0x00;
	 Paint_ClearWindows(1, 1, 1 + Font16.Width * 5, 1 + Font16.Height, WHITE);
	 Paint_DrawString_EN(1, 1,"START", &Font16, WHITE, BLACK);
	 EPD_1IN54_V2_DisplayPart(BlackImage);

	 Paint_ClearWindows(1, 67, 1 + Font16.Width * 5, 67 + Font16.Height, WHITE);
	 EPD_1IN54_V2_DisplayPart(BlackImage);
	}
	break;

	case 0x26:  if(comm == 1)
	{
	 event_flags->Error[1] = 0x00;
	 event_flags->k_50[1] = 0x01;
	 Paint_ClearWindows(1, 23, 1 + Font16.Width * 7, 23 + Font16.Height, WHITE);
	 Paint_DrawString_EN(1, 23,"STARTER", &Font16, BLACK, WHITE);
	 EPD_1IN54_V2_DisplayPart(BlackImage);
	}else if(comm == 0)
	{
	 event_flags->Error[1] = 0x00;
	 event_flags->k_50[1] = 0x00;
	 Paint_ClearWindows(1, 23, 1 + Font16.Width * 7, 23 + Font16.Height, WHITE);
	 Paint_DrawString_EN(1, 23,"STARTER", &Font16, WHITE, BLACK);
	 EPD_1IN54_V2_DisplayPart(BlackImage);
	}
	break;

	defult: Error_Handler();

	}
}

static void KEY_Data_Handler(uint8_t id, uint8_t* data)
{

	switch(id)
	{

	case 0x50:
	{
		if(event_flags->k_15[1] == 0x01)
	     {
		 uint8_t string_[6];
		 string_[5] = '\0';
		 for(uint8_t i = 0; i < 5; i++)
		 {
			 string_[i] = data[i + 1];
		 }
		 Paint_ClearWindows(1, 67, 1 + Font16.Width * 5, 67 + Font16.Height, WHITE);
		 Paint_DrawString_EN(1, 67, (char*)string_, &Font16, WHITE, BLACK);
		 EPD_1IN54_V2_DisplayPart(BlackImage);
		 }
	}

    break;

	defult: Error_Handler();

	}
}


void UART_Transmit(USART_TypeDef* UART,uint8_t* pData, uint32_t size)
{
	for(uint32_t i = 0; i < 3 ; i++)
	{
		while( (UART->SR & USART_SR_TXE) != USART_SR_TXE){}

		UART->DR = (uint8_t)start_uart[i];                     // start transmit
	}

	for(uint32_t i = 0; i < size ; i++)
	{
	while( (UART->SR & USART_SR_TXE) != USART_SR_TXE){}

	UART->DR = (uint8_t)pData[i];
	}

	for(uint8_t t = 0; t < 27; t++)
	{
		while( (UART->SR & USART_SR_TXE) != USART_SR_TXE){}

		UART->DR = (uint8_t)0xFF;
	}

	while( (UART->SR & USART_SR_TC) != USART_SR_TC){}

}

void UART_Receive(USART_TypeDef* UART, uint8_t* pData, uint32_t size)
{
	uint32_t cnt = size;
	while(cnt > 0)
	{

//	while( (UART->SR & USART_SR_RXNE) != USART_SR_RXNE){}

	*pData = (uint8_t)(UART->DR & (uint8_t)0xFF);

	pData++;
	cnt--;
	}


}

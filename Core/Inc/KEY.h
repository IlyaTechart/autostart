/*
 * KEY.h
 *
 *  Created on: Nov 23, 2024
 *      Author: I
 */
#include <stdint.h>
#include "stm32f411xe.h"

#ifndef INC_KEY_H_
#define INC_KEY_H_

#define ID_START_ENGINE          0x00U
#define COMMAND_START_ENGINE     0x01U

#define ID_STOP_ENGINE           0x01U
#define COMMAND_STOP_ENGINE      0x01U

#define ID_STATE_REQ             0x03U
#define COMMAND_STATE_REQ        0x01U

#define NUM_BUTTON_1             0x01U
#define NUM_BUTTON_2             0x02U

//===================================== FOR void Transmit_flags_choose(uint8_t num_flag) ========================================
#define NUMBER_OF_FLAG_TRANS_ACEPT        0x0U
#define NUMBER_OF_FLAG_K15                0x1U
#define NUMBER_OF_FLAG_K50                0x2U
#define NUMBER_OF_FLAG_TRANS_ERROR        0x3U
//===============================================================================================================================
typedef union
{
	struct
	{
		__IO uint8_t start_engine[2];
		__IO uint8_t stop_engine[2];

	}struct_command;

	__IO uint8_t ptr[4];

}KEY_Typedef_command;


typedef struct
{
	__IO uint8_t k_50[2]; // klemma 50
	__IO uint8_t k_15[2]; // klemma 15
	__IO uint8_t TransmitAcepted[2];
	__IO uint8_t Error[2];

}KEY_Typedef_flags;

typedef struct
{

}MON_Typedef_varieble;



extern void Error_Handler(void);

void KEY_Init(void);
void KEY_Press_Button(uint8_t num_button);
void KEY_ReciveData(void);
void KEY_Request_state(void);
void KEY_Event(uint8_t* command);
void UART_Transmit(USART_TypeDef* UART,uint8_t* pData, uint32_t size);
void UART_Receive(USART_TypeDef* UART, uint8_t* pData, uint32_t size);
#endif /* INC_KEY_H_ */

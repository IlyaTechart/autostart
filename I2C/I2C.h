/*
 * I2C.h
 *
 *  Created on: Dec 6, 2024
 *      Author: q
 */

#ifndef I2C_H_
#define I2C_H_
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "stm32f411xe.h"

void CMSIS_I2C_Reset(void);
void CMSIS_I2C1_Init(void);
bool CMSIS_I2C_Adress_Device_Scan(I2C_TypeDef *I2C, uint8_t Adress_Device, uint32_t Timeout_ms);
bool CMSIS_I2C_Data_Transmit(I2C_TypeDef *I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool CMSIS_I2C_Data_Receive(I2C_TypeDef *I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool CMSIS_I2C_MemWrite(I2C_TypeDef *I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool CMSIS_I2C_MemRead(I2C_TypeDef *I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);


#endif /* I2C_H_ */

/*
 * Lora.h
 *
 *  Created on: Nov 29, 2024
 *      Author: Khushi
 */

#ifndef INC_LORA_H_
#define INC_LORA_H_

#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include <stdio.h>

#include "main.h"			/* Cube Generated Init Code */


#define LORA_MODE_M0_RESET			HAL_GPIO_WritePin(MODE0_GPIO_Port, MODE0_Pin, GPIO_PIN_RESET)
#define LORA_MODE_M0_SET			HAL_GPIO_WritePin(MODE0_GPIO_Port, MODE0_Pin, GPIO_PIN_SET)
#define LORA_MODE_M1_RESET			HAL_GPIO_WritePin(MODE1_GPIO_Port, MODE1_Pin, GPIO_PIN_RESET)
#define LORA_MODE_M1_SET			HAL_GPIO_WritePin(MODE1_GPIO_Port, MODE1_Pin, GPIO_PIN_SET)


#define LED1_ON						HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED1_OFF					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED2_ON						HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)
#define LED2_OFF					HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define LED3_ON						HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET)
#define LED3_OFF					HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET)
#define LED3_TOGGLE					HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin)


/* DEBUG UART */
HAL_StatusTypeDef port_debug_receive_wait( uint8_t * data, uint16_t len, uint16_t * rx_len );
void port_debug_print(char * data);

HAL_StatusTypeDef port_lora_receive_wait( uint8_t * data1, uint16_t len1, uint16_t * rx_len1 );
void port_lora_print(char * data1);


#endif /* INC_LORA_H_ */

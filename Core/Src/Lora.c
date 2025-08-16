/*
 * Lora.c
 *
 *  Created on: Nov 29, 2024
 *      Author: Khushi
 */


#include "main.h"
#include "Lora.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef hlpuart1;

/* DEBUG UART */
HAL_StatusTypeDef port_debug_receive_wait( uint8_t * data, uint16_t len, uint16_t * rx_len )
{
	return HAL_UARTEx_ReceiveToIdle(&huart2, data, len, rx_len, 10000);
}

void port_debug_print(char * data)
{
	uint16_t len = (uint16_t)strlen((const char *)data);

	HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 500);
}
/*------------------------------------------------------------------*/

HAL_StatusTypeDef port_lora_receive_wait( uint8_t * data1, uint16_t len1, uint16_t * rx_len1 )
{
	return HAL_UARTEx_ReceiveToIdle(&hlpuart1, data1, len1, rx_len1, 10000);
}

void port_lora_print(char * data1)
{
	uint16_t len1 = (uint16_t)strlen((const char *)data1);

	HAL_UART_Transmit(&huart2, (uint8_t *)data1, len1, 500);
}

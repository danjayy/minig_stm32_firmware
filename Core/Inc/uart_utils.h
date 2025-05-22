/*
 * uart_utils.h
 *
 *  Created on: May 22, 2025
 *      Author: DEBORAH DANJUMA
 */

#ifndef INC_UART_UTILS_H_
#define INC_UART_UTILS_H_

#include "stm32g4xx_hal.h" // Include HAL for UART types

// Declare global variables as extern
extern uint8_t rx_buffer[256];
extern uint16_t rx_index;
extern uint8_t rx_data;
extern volatile uint8_t rx_complete;

// Function prototypes
void startUartReceive(UART_HandleTypeDef *huart);
uint16_t readLine(UART_HandleTypeDef *huart, uint8_t *data, uint16_t max_len);

#endif /* INC_UART_UTILS_H_ */

#include "stm32l4xx_hal.h"

extern UART_HandleTypeDef huart2;

/**
* @brief Transmits a single character over USART
* @param b_char - char to transmit
*/
void USART_putc(uint8_t b_char){
	HAL_UART_Transmit(&huart2, &b_char, 1, 100);
}

/**
* @brief Transmits a string of characters over USART
* @param b_string - pointer to array of characters
*/
void USART_puts(char *b_string){
	uint32_t i = 0;
	while(b_string[i] != '\0'){
		USART_putc(b_string[i]);
		i++;
	}
}


#include "stm32l4xx_hal.h"
#include "usart_com.h"

extern UART_HandleTypeDef huart2;

/**
* @brief Transmits a single character over USART
* @param b_char - char to transmit
*/
void USART_putc(uint8_t b_char){
	HAL_UART_Transmit(&huart2, &b_char, 1, USART_TRANSMIT_TIMEOUT);
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

/**
* @brief Transmits a single word (LSB firts) over USART 
* @param w_word - 32bit word to transmit
*/
void USART_putw(uint32_t w_word){
	
	for (uint32_t i = 0; i < 4; i++){
		// transmits word LSB first
		USART_putc((uint8_t)(w_word >> (i*8))); 
	}
}

/**
* @brief Transmits a single half word (LSB firts) over USART 
* @param hw_halfWord - 16bit half word to transmit
*/
void USART_puthw(uint16_t hw_halfWord){
	
	for (uint32_t i = 0; i < 2; i++){
		// transmits word LSB first
		USART_putc((uint8_t)(hw_halfWord >> (i*8))); 
	}
}

/**
* @brief Transmits a multiple words over USART
* @param aw_words - array of 32bit words to transmit
*/
void USART_putws(uint32_t *aw_words, uint32_t size){
	for (uint32_t i = 0; i < size; i++){
		USART_putw(aw_words[i]);
	}
}

/**
* @brief Receives a single character over USART
* @retval b_char - char to receive
*/
uint8_t USART_getc(void){
	
	uint8_t b_char[1] = {0};
	HAL_UART_Receive(&huart2, b_char, 1, USART_RECEIVE_TIMEOUT);
	
	return b_char[0];
}

/**
* @brief Receives a single word over USART
* @retval w_word - 32bit word to receive
*/
uint32_t USART_getw(void){
	
	uint8_t  rec_buf[4];
	uint32_t w_retWord;
	
	HAL_UART_Receive(&huart2, rec_buf, 4, USART_RECEIVE_TIMEOUT*4);
	
	// receives word LSB first
	for (uint32_t i = 0; i < 4; i++){
		w_retWord |= rec_buf[i] << i*8;
	}
	
	return w_retWord;
}



#include "stm32l4xx_hal.h"

/* Definitions */
#define USART_TRANSMIT_TIMEOUT 100
#define USART_RECEIVE_TIMEOUT  100

/*Function Prototypes*/
void USART_putc(uint8_t b_char);
void USART_puts(char *b_string);
void USART_putw(uint32_t w_word);
void USART_puthw(uint16_t hw_halfWord);
uint8_t USART_getc(void);
uint32_t USART_getw(void);
void USART_putws(uint32_t *aw_words, uint32_t size);

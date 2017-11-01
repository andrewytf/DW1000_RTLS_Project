#include "usart_com.h"
#include "deca_com.h"

//#include "deca_com.h"

/**
* @brief DW reset
*/
void reset_DW1000(void)
{
	
	// Enable GPIO PB5 used for DW1000 reset
	//GPIOB->MODER &= ~(0x3 << 10);
	GPIOB->MODER |= 0x3 << 10; // set PB5 to analog mode
	GPIOB->MODER |= 0x1 << 10; // set PB5 to output

	//drive the RSTn pin low
	GPIOB->ODR &=  ~(0x1 << 5);

	HAL_Delay(2); // wait 2 ms
	
	//put the pin back to tri-state ... as input
	//GPIOB->MODER &= ~(0x3 << 10); // set PB5 to input mode
	GPIOB->MODER |= 0x3 << 10; // set PB5 to analog mode
	
  HAL_Delay(2); // wait 2 ms
	
}

/**
* @brief DW wakeup
*/
void wakeup_DW1000(void){
	GPIOB->ODR |= 0x1 << 3; // set PB3 -> WAKEUP
	HAL_Delay(10);
}



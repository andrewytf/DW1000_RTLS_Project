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

/**
* @brief The main state machine
*/
void app_deca(){
	static uint32_t state = DECA_INIT;
	
	switch (state){
		case DECA_INIT:
			// perform initialization
			// determine next state to run depending on Anchor/Tag mode
			break;
		
		case DECA_TXBLINK_WAIT_SEND:
			// set up message frame control data and fill the rest of 
			// the message with tag address
			break;
		case DECA_TXPOLL_WAIT_SEND:
			// send poll message
			break;
		case DECA_RXE_WAIT:
			// this is pre-receiver enable state
			break;
		case DECA_RX_WAIT_DATA:
			// this state handles received data
			break;
		case DECA_SLEEP_DONE:
			// wake up deca from deep sleep
			break;
		case DECA_TXE_WAIT:
			// tag checks if it needs to go to sleep. 
			// If tag came here from sleep it will proceed to send next poll or blink
			break;
		case DECA_TXFINAL_WAIT_SEND:
			// here we send the Final message
			// it should include TX TS of poll, RX TS of anchor response and
			// (predicted/calculated) TX TS of final message with antenna delay 
			break;
		case DECA_TX_WAIT_CONF:
			break;
		default:
			// this state shoul not occur
			break;
	}
}

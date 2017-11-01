#include "stm32l4xx_hal.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi.h"

extern SPI_HandleTypeDef hspi1;

/**
* @fn 	 writetospi
* @brief Write to SPI - platform specific function for DecaWave. This function is used by DW API for SPI communication.
* @param headerLength - length of the header buffer
* @param headerBuffer - pointer to the header buffer byte array
* @param bodylength   - length of the data buffer
* @param bodyBuffer   - pointer to the data buffer byte array
* @retval Success flag. Can either be DWT_SUCCESS = 0 or DWT_ERROR = -1.
*/
int writetospi(uint16 headerLength, const uint8_t *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer){
	
	decamutexon();
	GPIOB->ODR &= ~(0x1 << 6); // Set PB6 (SPI_CS) low
	
	// dummy buffers (ignored)
	uint8 dummy_buffer1[headerLength];
	uint8 dummy_buffer2[bodylength];
	
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)headerBuffer, dummy_buffer1, headerLength, 5);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)bodyBuffer, dummy_buffer2, bodylength, 5);
	
	GPIOB->ODR |= 0x1 << 6; // Set PB6 (SPI_CS) high
	decamutexoff(0);
	
	return 0;
}

/**
* @fn 	 writetospi
* @brief Read from SPI - platform specific function for DecaWave. This function is used by DW API for SPI communication.
* @param headerLength - length of the header buffer
* @param headerBuffer - pointer to the header buffer byte array
* @param readlength   - length of the data buffer
* @param readBuffer   - pointer to the data buffer byte array
* @retval Success flag. Can either be DWT_SUCCESS = 0 or DWT_ERROR = -1.
*/
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer){
	decamutexon();
	GPIOB->ODR &= ~(0x1 << 6); // Set PB6 (SPI_CS) low
	
	// dummy buffers (ignored)
	uint8 dummy_buffer1[headerLength];
	uint8 dummy_buffer2[readlength];
	
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)headerBuffer, dummy_buffer1, headerLength, 5);
	HAL_SPI_TransmitReceive(&hspi1, dummy_buffer2, (uint8_t *)readBuffer, readlength, 5);
	
	GPIOB->ODR |= 0x1 << 6; // Set PB6 (SPI_CS) high
	decamutexoff(0);
	
	return 0;
}


/**
* @fn 	 decamutexoff
* @brief This function is used to restore the DW1000’s interrupt state as returned by decamutexon() function
				 This is called at the start of the critical section of SPI access.
* @param s - state of the target microcontroller's interrupt logic as it was on entry to the decamutexon function
*/
void decamutexoff (decaIrqStatus_t s){
	
	// TODO : save interrupt status
	
	//EXTI->RTSR1 |= 0x1; // enable rising edge interrupt
	EXTI->IMR1 |= 0x1; // EXTI0 intrrupt not masked
	
		
}


/**
* @fn 	 decamutexon
* @brief This function is used to turn on mutual exclusion (e.g. by disabling interrupts). 
				 This is called at the start of the critical section of SPI access.
* @retval s - state of the target microcontroller's interrupt logic as it was on entry to the decamutexon function
*/
int decamutexon (void){
	
	//EXTI->RTSR1 &= ~0x1; // disable rising edge interrupt
	EXTI->IMR1 &= ~0x1; // mask EXTI0 interrupt
	// TODO : return interrupt status before disabling
	return 0;
}


/**
* @fn 	 deca_sleep
* @brief This function is used to wait for a given amount of time before proceeding to the next step of the calling function.
* @param time_ms - The amount of time to wait, expressed in milliseconds.
*/
void deca_sleep (unsigned int time_ms){
	HAL_Delay(time_ms);
}

void set_spi_speed_high(void){
	
	SPI1->CR1 &= ~(0x7 << 3);
	SPI1->CR1 |= (0x1 << 3); // set divider to 4
	
}

void set_spi_speed_low(void){
	
	SPI1->CR1 &= ~(0x7 << 3);
	SPI1->CR1 |= (0x4 << 3); // set divider to 32
	
}

void spi_config(void){
	SPI1->CR1 &= ~0x3;
	
	//SPI1->CR1 |= 0x1;
	//SPI1->CR1 |= 0x2;
}


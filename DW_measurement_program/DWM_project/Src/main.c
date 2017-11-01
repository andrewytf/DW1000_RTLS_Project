/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_com.h"
#include "usart_com.h"
#include "deca_spi.h"
#include "deca_meas.h"

#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint32_t dwt_state = DECA_STATE_IDLE;
uint32_t dwt_state_local = DECA_STATE_IDLE;

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = 
	{ // working configuration
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_4096,   /* Preamble length. Used in TX only. */
    DWT_PAC64,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_SFDTOC_DEF /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
	};
	
static dwt_config_t def_config = 
	{ // working configuration
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_4096,   /* Preamble length. Used in TX only. */
    DWT_PAC64,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_SFDTOC_DEF /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
	};
	
	
	// DW transmission diagnostic structure
	static dwt_rxdiag_t rx_diag;
	
/* Configuration message
* byte 0 - message type
* byte 1 - channel number
* byte 2 - PRF
* byte 3 - Data Rate
* byte 4 - Preamble sequence length
* byte 5 - payload size
*/
// default configuration message
uint8_t ac_defConfigMsg[6] = {MSG_TYPE_CONF_SET, 2, DWT_PRF_64M, DWT_BR_110K, DWT_PLEN_1024, PAYLOAD_20B_CODE};

/* Default antenna delay values for 64 MHz PRF */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Frames used for measurement process*/
static uint8 ac_txConfMsg[CONFIG_MSG_LENGTH] = {MSG_TYPE_CONF_SET, 2, 2, DWT_BR_110K, DWT_PLEN_1024, PAYLOAD_20B_CODE, 0, 0, 0, 0};
static uint8 ac_txConfigConfirm[CONFIG_MSG_LENGTH] = {MSG_TYPE_CONF_CONFIRM, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 ac_pollMsg[PAYLOAD_1000B]; // set size to max possible payload
static uint8 ac_respMsg[PAYLOAD_1000B]; // set size to max possible payload

/* Payload size */
uint32 w_dwtPayloadSize = PAYLOAD_20B;

/* Data rate */
uint32_t w_dataRate = 110;

/* Receiver timeout value */
uint32_t w_timeoutVal = 10000;

/* Transmit and received messages counters */
uint8 w_trMsgCnt = 0;      	          	// transmit messages counter
uint32_t w_recMsgCnt = 0;		 			          // receive messages counter
uint32_t w_rxRespTimeoutCnt = 0;            // receive response timeount counter
uint32_t w_rxConfigTimeoutCnt = 0;          // receive configuration timeount counter
uint32_t w_rxConfigConfirmTimeoutCnt = 0;   // receive configuration confirm timeount counter
uint32_t w_rxErrCnt = 0;					          // error receiving counter
uint32_t w_trCallbackCnt = 0;			          // transmit callback counter
uint32_t w_rxPollTimeoutCnt = 0;            // receive poll timeout counter

uint32_t w_rxTagRecTimeoutCnt = 0;

/* Timer values */
uint32_t w_tagTimVal = 0;   		// Tag message processing time
uint32_t w_anchorRxTimVal = 0;  // Anchor receive message time

/* Measurement data for host*/
uint32_t aw_measTimValues[NUM_MEASURE_MESSAGES];       // Measured Timer values
uint32_t aw_measTurnaroundTimes[NUM_MEASURE_MESSAGES]; // Tag's measured turnaround times
uint32_t aw_ChImpulseResponse[NUM_MEASURE_MESSAGES];   // Channel impulse response (used for RSSI calculation)
uint32_t aw_rxPreamCount[NUM_MEASURE_MESSAGES];        // Receive preamble counter (used for RSSI calculation)

/* DW module mode*/
/* Set desired operating mode for DW here */
uint32_t w_dwtMode = DECA_MODE_TAG;

/* Receive buffer */
static uint8 ac_rxBuffer[PAYLOAD_1000B]; // Set receive buffer length to maximum possible payload

/* Function declaration */
void setConfigurationStruct(uint8 *ac_configMsg, dwt_config_t *dwtStructConfig);
uint8 getPACsize(uint8 c_PrmLength);
uint8 getPRMcode(uint8 c_chNum, uint8 c_PRF);
uint8 getPHR(uint8 payload);
uint32_t getPayload(uint8 payloadCode);
uint32_t getDataRateFromCode(uint8 c_dataRateCode);
uint32_t getTimeoutVal(uint8 c_timoutCode);
void sendMeasurementData(void);
void clearArray(uint32_t *aw_array, uint32_t w_size);
uint32_t computeVarianceSum(uint32_t *aw_wordArray, uint32_t w_size, uint8_t c_excludeZerosEn);
uint32_t computeAverage(uint32_t *aw_wordArray, uint32_t w_size, uint8_t c_excludeZerosEn);
uint32_t computeSum(uint32_t *aw_wordArray, uint32_t w_size);

// declaration of callback functions
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_to_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);
static void tx_conf_cb(const dwt_cb_data_t *cb_data);

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
	wakeup_DW1000();
	//USART_puts("DWT wakeup\n");
	
	reset_DW1000();
	//USART_puts("DWT reset\n");
	HAL_Delay(5);
	
	/* Initialize DWT*/
	set_spi_speed_low(); // set SPI to low speed (2.5MHz) for initialization
	if (dwt_initialise(DWT_LOADUCODE) == DWT_SUCCESS){
		//USART_puts("Initialization OK\n");
	} 
	else 
	{
		//USART_puts("Initialization FAILED\n");
		while(1){};
	}
	set_spi_speed_high(); // set SPI to high speed (20MHz)
	
	HAL_Delay(5);
	
	/* Configure DW1000. */
	dwt_configure(&config);
	//USART_puts("DW configured\n");
	HAL_Delay(5);
	
	/* Register RX call-back. */
	dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);

	/* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
	dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);

	// set delay after transmiting
	dwt_setrxaftertxdelay(TX_TO_RX_DELAY_UUS);

	/* Apply default antenna delay value */
	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);

	// Check SPI communication by reading ID
	while(dwt_readdevid() != DWT_DEVICE_ID){
		//USART_puts("ID NOT read successfully\n");
		while(1){};
	}
	//USART_puts("ID read successfully\n");
	
	// TAG should start receiving on power on.
	if (w_dwtMode == DECA_MODE_TAG ){
		//USART_puts("Selected mode is TAG - RESPONDER\n");
		
		/* Activate reception immediately. */
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
		dwt_state = DECA_TAG_RECEIVING_STATE;
		
		/* Reset and start timer */
		TIM2->CR1 |= 0x1; // Timer enable
		TIM2->EGR |= 0x1; // Update event
		
		/* Set receive timeout */
		dwt_setrxtimeout(DWT_TAG_RECEIVE_TIMEOUT); 
	}
	else if (w_dwtMode == DECA_MODE_ANCHOR){
		//USART_puts("Selected mode is ANCHOR - INITIATOR\n");
		
		dwt_state = DECA_ANCHOR_STATE_IDLE;
		
		TIM2->CR1 |= 0x1; // Timer enable
		TIM2->EGR |= 0x1; // Update event
		
		/* Set receive timeout */
		dwt_setrxtimeout(DWT_ANCHOR_RECEIVE_TIMEOUT); 
	}
	
	/* Alert host, that MCU reset has occured*/
	USART_putc(MCU_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	
		dwt_state_local = dwt_state;
		
		/* Main state machine*/
		switch (dwt_state_local){
			case DECA_ANCHOR_STATE_IDLE:
				//USART_puts("DECA_ANCHOR_IDLE\n");
				if (USART_getc() == NEW_CONFIGURATION_INCOMING){
					
					EXTI->IMR1 &= ~0x1; // mask EXTI0 interrupt
					
					/* Create configuration message from expected bytes */
					for (uint32_t i = 0; i < USART_NUM_OF_CONFIG; i++){
						ac_txConfMsg[i+1] = USART_getc(); 
					}
					
					ac_txConfMsg[MSG_TYPE_BYTE] = MSG_TYPE_CONF_SET;
					
					/* Reset all counters */
					w_trMsgCnt = 0;      	             // Transmit messages counter
					w_recMsgCnt = 0;		 			         // Receive messages counter
					w_rxRespTimeoutCnt = 0;            // Receive response timeout counter
					w_rxConfigTimeoutCnt = 0;			  	 // Receive configuration confirm timeout counter
					w_rxConfigConfirmTimeoutCnt = 0; 	 // Receive configuration confirm timeout counter
					w_rxErrCnt = 0;					  		  	 // Error receiving counter
					w_trCallbackCnt = 0;						   // Transmit callback counter
					w_rxPollTimeoutCnt = 0;   		   	 // Receive poll timeout counter
					
					/* Send configuration to TAG */
					/* Write frame data to DW1000 and prepare transmission. */
					dwt_writetxdata(sizeof(ac_txConfMsg), ac_txConfMsg, 0); /* Zero offset in TX buffer. */
					dwt_writetxfctrl(sizeof(ac_txConfMsg), 0, 1);				    /* Zero offset in TX buffer,  ranging. */

					/* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
					dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
					//USART_puts("Configuration sent to TAG\n");
					
					dwt_state = DECA_ANCHOR_RECEIVING_CONFIRM_STATE;
					
					EXTI->IMR1 |= 0x1; // unmask EXTI0 interrupt

				}
				
				/* Reset and start timer */
				TIM2->EGR |= 0x1;
				
				break;
				
			case DECA_TAG_CONFIG_RECEIVED_STATE:
				//USART_puts("DECA_TAG_CONFIG_RECEIVED_STATE\n");
			
				EXTI->IMR1 &= ~0x1; // mask EXTI0 interrupt
			
				/* Send confirmation message to Anchor */
				/* Write frame data to DW1000 and prepare transmission. */
				ac_txConfigConfirm[MSG_TYPE_BYTE] = MSG_TYPE_CONF_CONFIRM;
				dwt_writetxdata(sizeof(ac_txConfigConfirm), ac_txConfigConfirm, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(ac_txConfigConfirm), 0, 1);				          /* Zero offset in TX buffer,  ranging. */

				/* Start transmission */
				dwt_starttx(DWT_START_TX_IMMEDIATE);
			
				/* Delay here, so message is sent before we force transmitter off */
				HAL_Delay(5);
			
				/* Disable receiver, transmitter, so new configuration can be set */
				dwt_forcetrxoff();
				dwt_rxreset();
				
				/* Set configuration */
				setConfigurationStruct(ac_rxBuffer, &config);
				dwt_configure(&config);
				
				/* Set payload size */
				w_dwtPayloadSize = getPayload(ac_rxBuffer[PAYLOAD_SIZE_BYTE]);
				
				/* Reset receive poll counter */
				w_rxPollTimeoutCnt = 0;
				
				/* Reset receive error counter */
				w_rxErrCnt = 0;
				
				/* Reset counter */
				w_rxTagRecTimeoutCnt = 0;
				
				/* Activate reception */
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
				
				dwt_state = DECA_TAG_RECEIVING_STATE;

				/* Reset and start timer */
				TIM2->EGR |= 0x1;
				
				EXTI->IMR1 |= 0x1; // unmask EXTI0 interrupt
					
				break;
				
			case DECA_ANCHOR_CONFIRM_RECEIVED_STATE:
				//USART_puts("DECA_ANCHOR_CONFIRM_RECEIVED_STATE\n");
			
				EXTI->IMR1 &= ~0x1; // mask EXTI0 interrupt

				/* Disable receiver, transmitter, so new configuration can be set. */
				dwt_forcetrxoff();
				dwt_rxreset();

				/* Set configuration */
				setConfigurationStruct(ac_txConfMsg, &config); // set config structure
				dwt_configure(&config); 								 // set dwt configuration
				
				/* Get payload from received configuration message */
				w_dwtPayloadSize = getPayload(ac_txConfMsg[PAYLOAD_SIZE_BYTE]);
				
				/* Reset transmit and receive counters */
				w_recMsgCnt = 0;
				w_trMsgCnt = 0;
			
				w_rxConfigConfirmTimeoutCnt = 0; 
			
				/* Wait for tag to properly configure */
				HAL_Delay(5);
			
				/* Start with measuring exchange */
				/* Write frame data to DW1000 and prepare transmission. */
				ac_pollMsg[MSG_TYPE_BYTE] = MSG_TYPE_MEAS_POLL;
				ac_pollMsg[POLL_NUM_BYTE] = 0;
				dwt_writetxdata(w_dwtPayloadSize, ac_pollMsg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(w_dwtPayloadSize, 0, 1);		     	/* Zero offset in TX buffer,  ranging. */

				/* Reset and start Timer */
				TIM2->CR1 |=0x1;
				TIM2->EGR |= 0x1;

				/* Increment transmit counter value */
				w_trMsgCnt++;
				
				/* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
				
				dwt_state = DECA_ANCHOR_RECEIVING_RESPONSE_STATE;
				
				EXTI->IMR1 |= 0x1; // unmask EXTI0 interrupt

				break;
				
			case DECA_TAG_POLL_RECEIVED_STATE:
				
				EXTI->IMR1 &= ~0x1; // mask EXTI0 interrupt
			
				//USART_puts("DECA_TAG_POLL_RECEIVED_STATE\n");
			
				/* Send response */
				/* Write frame data to DW1000 and prepare transmission. */
				ac_respMsg[MSG_TYPE_BYTE] = MSG_TYPE_MEAS_RESPONSE;
				w_trMsgCnt = ac_rxBuffer[POLL_NUM_BYTE];
				
				/* Write previous receive timestamp to message, little endian*/
				for (uint32_t i = 0; i < 4;  i++){
					ac_respMsg[TIMESTAMP_OFFSET + i] = (uint8)(w_tagTimVal >> 8*i);
				}
				
				ac_respMsg[POLL_NUM_BYTE] = w_trMsgCnt;
				
				dwt_writetxdata(w_dwtPayloadSize, ac_respMsg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(w_dwtPayloadSize, 0, 1);				  /* Zero offset in TX buffer,  ranging. */
				
				/* Save interval from receiving message interrupt to before sending response. */
				w_tagTimVal = TIM2->CNT;
				
				/* Reset and start timer */
				TIM2->EGR |= 0x1;
				
				/* Reset counter */
				w_rxTagRecTimeoutCnt = 0;
				
				/* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
				//USART_puts("Response sent\n");

				dwt_state = DECA_TAG_RECEIVING_STATE;

				EXTI->IMR1 |= 0x1; // unmask EXTI0 interrupt
				
				break;

			case DECA_ANCHOR_RESPONSE_RECEIVED_STATE:
				
				EXTI->IMR1 &= ~0x1; // mask EXTI0 interrupt
			
				//USART_puts("DECA_ANCHOR_RESPONSE_RECEIVED_STATE\n");
				
				if (ac_rxBuffer[POLL_NUM_BYTE] != w_trMsgCnt-1){
					dwt_state = DECA_ANCHOR_REC_RESPONSE_TMOUT_STATE;
					
					EXTI->IMR1 |= 0x1; // unmask EXTI0 interrupt
					break;
				}
				
				/* Reset configuration confirm timeout counter */
				w_rxConfigConfirmTimeoutCnt = 0; 
				
				/* Read receiver diagnostics from DW */
				dwt_readdiagnostics(&rx_diag);
			
				/* Save timer value */
				//aw_measTimValues[w_trMsgCnt-1] = w_anchorRxTimVal;
				aw_measTimValues[w_recMsgCnt] = w_anchorRxTimVal;

				/* Subtract Tag's internal timestamp value from previous recorded Anchor value */
				if (w_recMsgCnt > 0){
					
					/* Read received Tag's timestamp value */
					w_tagTimVal = 0;
					for (uint32_t i = 0; i < 4; i++){
						w_tagTimVal |= (uint32_t)(ac_rxBuffer[TIMESTAMP_OFFSET + i] << 8*i);
					}
					
					aw_measTurnaroundTimes[w_recMsgCnt] = w_tagTimVal;
					
					aw_measTimValues[w_recMsgCnt] = (uint32_t)(aw_measTimValues[w_recMsgCnt] - w_tagTimVal);
				}
		
				/* Get values for computing RSSI */
				aw_ChImpulseResponse[w_recMsgCnt] = (uint32_t)rx_diag.maxGrowthCIR;
				aw_rxPreamCount[w_recMsgCnt] = (uint32_t)rx_diag.rxPreamCount;

				// increment receive counter value
				w_recMsgCnt++;
			
				if (w_trMsgCnt < NUM_MEASURE_MESSAGES){
					/* send poll again */
					/* Write frame data to DW1000 and prepare transmission. */
					ac_pollMsg[MSG_TYPE_BYTE] = MSG_TYPE_MEAS_POLL;
					ac_pollMsg[POLL_NUM_BYTE] = w_trMsgCnt;
					dwt_writetxdata(w_dwtPayloadSize, ac_pollMsg, 0); /* Zero offset in TX buffer. */
					dwt_writetxfctrl(w_dwtPayloadSize, 0, 1);				  /* Zero offset in TX buffer,  ranging. */
					
					/* Reset and start timer */
					TIM2->CR1 |= 0x1; // Timer enable
					TIM2->EGR |= 0x1;	// Update event generation
					
					/* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
					dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
						
					/* Increment transmit counter value */
					w_trMsgCnt++;
					
					dwt_state = DECA_ANCHOR_RECEIVING_RESPONSE_STATE;
					
				}
				else {
					//USART_puts("Measurement complete\n");

					/* Send measurement data to host computer */
					sendMeasurementData();
					
					/* Set Anchor back to IDLE state, where he will wait for instructions from host */
					dwt_state = DECA_ANCHOR_STATE_IDLE;
					
				}
				
				EXTI->IMR1 |= 0x1; // unmask EXTI0 interrupt
				
				break;
			
			case DECA_UNKNOWN_RECEIVED_STATE:
				
				EXTI->IMR1 &= ~0x1; // mask EXTI0 interrupt
			
				if (w_dwtMode == DECA_MODE_TAG){
					
					/* Start receiving again immediately */
					dwt_rxenable(DWT_START_RX_IMMEDIATE);
					
					dwt_state = DECA_TAG_RECEIVING_STATE;

				}
				else if (w_dwtMode == DECA_MODE_ANCHOR){
					
					/* Set to idle state and alert host about error */
					dwt_forcetrxoff(); // Force DW transmiter/receiver off
					dwt_rxreset();
					
					// Alert host that configuration was unable to deliver (too many timeouts occured)
					//USART_putc(MEAS_UNKNOWN_REC_ERROR);
					
					// Reset configuration to default
					//setConfigurationStruct(ac_defConfigMsg, &config);
					dwt_configure(&def_config);
					
					w_dwtPayloadSize = PAYLOAD_20B;
					
					/* Wait for Tag to reconfigure to default configuration */
					HAL_Delay(RX_POLL_TIMEOUT_CNT_MAX * DWT_TAG_RECEIVE_TIMEOUT / 1000); 
					
					/* Reset and start timer */
					TIM2->EGR |= 0x1;
					
					//dwt_state = DECA_ANCHOR_STATE_IDLE;
					/* Send configuration again */
					/* Write frame data to DW1000 and prepare transmission. */
					dwt_writetxdata(sizeof(ac_txConfMsg), ac_txConfMsg, 0); /* Zero offset in TX buffer. */
					dwt_writetxfctrl(sizeof(ac_txConfMsg), 0, 1);				    /* Zero offset in TX buffer,  ranging. */

					/* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
					dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
					//USART_puts("Configuration sent to TAG\n");

					dwt_state = DECA_ANCHOR_RECEIVING_CONFIRM_STATE;
					
					w_rxConfigConfirmTimeoutCnt = 0; 

				}
				
				/* Reset and start timer */
				TIM2->EGR |= 0x1;
				
				EXTI->IMR1 |= 0x1; // unmask EXTI0 interrupt
				
				break;
			
			case DECA_ANCHOR_REC_CONFIRM_TMOUT_STATE:
				
				EXTI->IMR1 &= ~0x1; // mask EXTI0 interrupt
			
				//USART_puts("DECA_ANCHOR_REC_CONFIRM_TMOUT_STATE\n");
				
				w_rxConfigConfirmTimeoutCnt++;
			
				if (w_rxConfigConfirmTimeoutCnt > RX_CONF_TIMEOUT_CNT_MAX){
					
					dwt_forcetrxoff(); // force DW transmiter/receiver off
					dwt_rxreset();

					/* Reset configuration to default */
					//setConfigurationStruct(ac_defConfigMsg, &config);
					dwt_configure(&def_config);
					
					w_dwtPayloadSize = PAYLOAD_20B;
					
					/* Reset receive configuration confirm timeout counter */
					w_rxConfigConfirmTimeoutCnt = 0;
					
					/* Wait for Tag to reconfigure to default configuration */
					HAL_Delay(RX_POLL_TIMEOUT_CNT_MAX * DWT_TAG_RECEIVE_TIMEOUT / 1000); 

					/* Reset and start timer */
					TIM2->EGR |= 0x1;
					
					/* Alert host that configuration was unable to deliver (too many timeouts occured) */
					USART_putc(MEAS_ERROR_CONFIG);
					
					dwt_state = DECA_ANCHOR_STATE_IDLE;

				}
				else {
					/* Send configuration again */
					/* Write frame data to DW1000 and prepare transmission. */
					ac_txConfMsg[MSG_TYPE_BYTE] = MSG_TYPE_CONF_SET;
					dwt_writetxdata(sizeof(ac_txConfMsg), ac_txConfMsg, 0); /* Zero offset in TX buffer. */
					dwt_writetxfctrl(sizeof(ac_txConfMsg), 0, 1);				    /* Zero offset in TX buffer,  ranging. */

					/* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
					dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
					
					/* Reset and start timer */
					TIM2->EGR |= 0x1;
					
					dwt_state = DECA_ANCHOR_RECEIVING_CONFIRM_STATE;
					
					//dwt_state = DECA_ANCHOR_SENDING_CONFIG_STATE;
					
				}
				
				EXTI->IMR1 |= 0x1; // unmask EXTI0 interrupt
				
				break;
				
			case DECA_ANCHOR_REC_RESPONSE_TMOUT_STATE:	
				
				EXTI->IMR1 &= ~0x1; // mask EXTI0 interrupt
			
				//USART_puts("DECA_ANCHOR_REC_RESPONSE_TMOUT_STATE\n");
				
				/* Increment receive respond timeout value */
				w_rxRespTimeoutCnt++;
				
				if (w_rxRespTimeoutCnt > RX_RESP_TIMEOUT_CNT_MAX){
					
					dwt_forcetrxoff(); // force DW transmiter/receiver off
					dwt_rxreset();
					
					/* Reset configuration to default */
					//setConfigurationStruct(ac_defConfigMsg, &config);
					dwt_configure(&def_config);
					
					w_dwtPayloadSize = PAYLOAD_20B;
					
					/* reset receive response timeout counter */
					w_rxRespTimeoutCnt = 0;
					
					/* Wait for Tag to reconfigure to default configuration */
					HAL_Delay( RX_POLL_TIMEOUT_CNT_MAX * DWT_TAG_RECEIVE_TIMEOUT  / 1000); 
					
					/* Reset and start Timer */
					TIM2->EGR |= 0x1;
					
					/* Alert host that measurement was unable to complete (too many timeouts occured) */
					USART_putc(MEAS_ERROR_RESP);
					
					dwt_state = DECA_ANCHOR_STATE_IDLE;
					
				}
				else {
					/* Send poll again */
					/* Write frame data to DW1000 and prepare transmission */
					ac_pollMsg[MSG_TYPE_BYTE] = MSG_TYPE_MEAS_POLL;
					ac_pollMsg[POLL_NUM_BYTE] = w_trMsgCnt;
					
					dwt_writetxdata(w_dwtPayloadSize, ac_pollMsg, 0); /* Zero offset in TX buffer. */
					dwt_writetxfctrl(w_dwtPayloadSize, 0, 1);				  /* Zero offset in TX buffer,  ranging. */

					/* Reset and start Timer */
					TIM2->EGR |= 0x1;
					
					/* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
					dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

					/* Increment transmit counter value */
					w_trMsgCnt++;
					
					dwt_state = DECA_ANCHOR_RECEIVING_RESPONSE_STATE;
				}
				
				EXTI->IMR1 |= 0x1; // unmask EXTI0 interrupt
				
				break;
				
			case DECA_TAG_RECEIVING_TMOUT_STATE:
				
				EXTI->IMR1 &= ~0x1; // mask EXTI0 interrupt		

				w_rxTagRecTimeoutCnt++;			
			
				if (w_rxTagRecTimeoutCnt > RX_POLL_TIMEOUT_CNT_MAX){
					dwt_forcetrxoff(); // force DW transmiter/receiver off
					dwt_rxreset();
					
					/* Reset configuration to default */
					//setConfigurationStruct(ac_defConfigMsg, &config);
					dwt_configure(&def_config);
					
					HAL_Delay(5);
					
					w_dwtPayloadSize = PAYLOAD_20B;
					
					/* Reset counters */
					w_rxTagRecTimeoutCnt = 0;

				}
				
				/* Start receiving again immediately */
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
				
				dwt_state = DECA_TAG_RECEIVING_STATE;
				
				/* Reset and start timer */
				TIM2->EGR |= 0x1;
			
				EXTI->IMR1 |= 0x1; // unmask EXTI0 interrupt
				
				break;

			/* In case Anchor gets stuck, we check Timer value and reset Anchor back to IDLE state. 
			 * We also need to alert host that error occured.
			 */
			case DECA_ANCHOR_RECEIVING_RESPONSE_STATE:
			case DECA_ANCHOR_RECEIVING_CONFIRM_STATE:
			
				if (TIM2->CNT > TIM_CNT_TIMEOUT){
					
					/* Force DW transmiter/receiver off */
					dwt_forcetrxoff(); 
					dwt_rxreset();
					
					/* Reset configuration to default */
					//setConfigurationStruct(ac_defConfigMsg, &config);
					dwt_configure(&def_config);
					
					HAL_Delay(5);

					dwt_state = DECA_ANCHOR_STATE_IDLE;
					
					/* Reset and start timer */
					TIM2->EGR |= 0x1;
					
					USART_putc(MEAS_RX_ERROR);
					
				}
				break;
				
			case DECA_TAG_RECEIVING_STATE:
				if (TIM2->CNT > RX_POLL_TIMEOUT_CNT_MAX * DWT_TAG_RECEIVE_TIMEOUT){
					
					/* Force DW transmiter/receiver off */
					dwt_forcetrxoff(); 
					dwt_rxreset();
					
					/* Reset configuration to default */
					//setConfigurationStruct(ac_defConfigMsg, &config);
					dwt_configure(&def_config);
					
					HAL_Delay(5);
					
					/* Reset and start timer */
					TIM2->EGR |= 0x1;
					
					/* Reset counters */
					w_rxTagRecTimeoutCnt = 0;
					
					/* Start receiving again immediately */
					dwt_rxenable(DWT_START_RX_IMMEDIATE);
					
					dwt_state = DECA_TAG_RECEIVING_STATE;
				}
				
				break;
		}	
		
			
		
	} /* end while*/
	
  /* USER CODE END 3 */

} /* end main */

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 36;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  __PWR_CLK_ENABLE();

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  HAL_RCCEx_EnableMSIPLLMode();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLED;
  HAL_SPI_Init(&hspi1);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

}

/* USB_OTG_FS init function */
void MX_USB_OTG_FS_PCD_Init(void)
{

  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 7;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.ep0_mps = DEP0CTL_MPS_64;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  HAL_PCD_Init(&hpcd_USB_OTG_FS);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_ok_cb()
 *
 * @brief Callback to process RX good frame events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
	
	if (w_dwtMode == DECA_MODE_TAG){
		
		/* Reset and start Timer */
		TIM2->CR1 |= 0x1;
		TIM2->EGR |= 0x1;
		
	}
	else if (w_dwtMode == DECA_MODE_ANCHOR){

		w_anchorRxTimVal = TIM2->CNT;
		
	}
	
	int i;
	
	/* Clear local RX buffer to avoid having leftovers from previous receptions. This is not necessary but is included here to aid reading the RX
	 * buffer. */
	for (i = 0 ; i < w_dwtPayloadSize; i++ )
	{
		ac_rxBuffer[i] = 0;
	}

	/* A frame has been received, copy it to our local buffer. */
	if (cb_data->datalength <= PAYLOAD_1000B)
	{
		dwt_readrxdata(ac_rxBuffer, cb_data->datalength, 0);
	
		switch (ac_rxBuffer[MSG_TYPE_BYTE]){
			case MSG_TYPE_CONF_SET:
				dwt_state = DECA_TAG_CONFIG_RECEIVED_STATE;
				break;
			case MSG_TYPE_CONF_CONFIRM:
				dwt_state = DECA_ANCHOR_CONFIRM_RECEIVED_STATE;
				break;
			case MSG_TYPE_MEAS_POLL:
				dwt_state = DECA_TAG_POLL_RECEIVED_STATE;
				break;
			case MSG_TYPE_MEAS_RESPONSE:
				
				if (dwt_state == DECA_ANCHOR_RECEIVING_CONFIRM_STATE){
					dwt_state = DECA_ANCHOR_REC_CONFIRM_TMOUT_STATE;	
				}
				else{
					dwt_state = DECA_ANCHOR_RESPONSE_RECEIVED_STATE;
				}
				
				break;
				
			default:
				/* This state should not occur */
				if (dwt_state == DECA_ANCHOR_RECEIVING_RESPONSE_STATE){
					dwt_state = DECA_ANCHOR_REC_RESPONSE_TMOUT_STATE;
				}
				else if (dwt_state == DECA_ANCHOR_RECEIVING_CONFIRM_STATE){
					dwt_state = DECA_ANCHOR_REC_CONFIRM_TMOUT_STATE;
				}
				else if (dwt_state == DECA_TAG_RECEIVING_STATE){
					dwt_state = DECA_TAG_RECEIVING_TMOUT_STATE;
				}
				
				// dwt_state = DECA_UNKNOWN_RECEIVED_STATE;
		}
	}
	else {
		// start receiving again immediately
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_to_cb()
 * @brief Callback to process RX timeout events
 * @param  cb_data  callback data
 * @return  none
 */
static void rx_to_cb(const dwt_cb_data_t *cb_data)
{
	/* Timeout occured while receiving. Check DW state and react accordingly. */
	switch (dwt_state){
		case DECA_ANCHOR_RECEIVING_CONFIRM_STATE:
			dwt_state = DECA_ANCHOR_REC_CONFIRM_TMOUT_STATE;
			break;
		case DECA_ANCHOR_RECEIVING_RESPONSE_STATE:
			dwt_state = DECA_ANCHOR_REC_RESPONSE_TMOUT_STATE;
			break;
		case DECA_TAG_RECEIVING_STATE:
			dwt_state = DECA_TAG_RECEIVING_TMOUT_STATE;
			break;

	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_err_cb()
 * @brief Callback to process RX error events
 * @param  cb_data  callback data
 * @return  none
 */
static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
	/* Reset receiver and start receiving again */
	//dwt_rxreset();
	
		if (dwt_state == DECA_ANCHOR_RECEIVING_RESPONSE_STATE){
		dwt_state = DECA_ANCHOR_REC_RESPONSE_TMOUT_STATE;
		}
		else if (dwt_state == DECA_ANCHOR_RECEIVING_CONFIRM_STATE){
			dwt_state = DECA_ANCHOR_REC_CONFIRM_TMOUT_STATE;
		}
		else if (dwt_state == DECA_TAG_RECEIVING_STATE){
			dwt_state = DECA_TAG_RECEIVING_TMOUT_STATE;
		}
//		else if (dwt_state == DECA_TAG_RECEIVING_POLL_STATE){
//			dwt_state = DECA_TAG_REC_POLL_TMOUT_STATE;
//		}
//		else if (dwt_state == DECA_TAG_RECEIVING_CONFIG_STATE){
//			dwt_state = DECA_TAG_REC_CONFIG_TMOUT_STATE;
//		}
//	dwt_rxenable(DWT_START_RX_IMMEDIATE);
//	w_rxErrCnt++;
	
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tx_conf_cb()
 *
 * @brief Callback to process TX confirmation events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void tx_conf_cb(const dwt_cb_data_t *cb_data)
{

	//USART_puts("Transmit complete callback\n");
	/* Increment transmit callback counter value */
	w_trCallbackCnt++;

}

/**
* @fn setConfigurationStruct
* @brief Sets configuration structure according to received message.
* @param (uint8) ac_configMsg - pointer to 8bit configuration message array
* @param (dwt_config_t) dwtStructConfig - DW configuration structure
* Configuration message
* byte 0 - message type
* byte 1 - channel number
* byte 2 - PRF
* byte 3 - Data Rate
* byte 4 - Preamble sequence length
* byte 5 - payload size
*
*/
void setConfigurationStruct(uint8 *ac_configMsg, dwt_config_t *dwtStructConfig){

	uint8 c_chNum = ac_configMsg[CHANNEL_NUMBER_BYTE];
	uint8 c_PRF = ac_configMsg[PRF_BYTE];
	uint8 c_DataRate = ac_configMsg[DATA_RATE_BYTE];
	uint8 c_PrmLengthCode = ac_configMsg[PRM_LENGTH_BYTE];
	uint8 c_payloadCode = ac_configMsg[PAYLOAD_SIZE_BYTE];
	
	/* Set preamble codes as recommended in DW API Guide */
	uint8 c_prCode = getPRMcode(c_chNum, c_PRF);

	/* Get standard or extended PHR mode, according to payload */
	uint32_t w_payload = getPayload(c_payloadCode);
	uint8 c_PHR = getPHR(w_payload);
	
	/* Set PAC size as recommended in DW API Guide */
	uint8 c_PACsizeCode = getPACsize(c_PrmLengthCode);
	
	/* Set nsSFD 
	* NOTE: It has been discovered that 850kbps data rate doesnt 
	*				work with non - standard SFD. So if 110kbps or 6.8Mbps Data rate is used, 
	* 			non - standard SFD should be used, but if 850kbps is used, 
	*				standard SFD is required.
	*/
	//uint8 c_nsSFD = (uint8)(c_DataRate == 0 || c_DataRate == 2);
	uint8 c_nsSFD;
	if (c_DataRate == 0 || c_DataRate == 2){
		c_nsSFD = 1;
	}
	else {
		c_nsSFD = 0;
	}
	
	/* Set configuration strcture fields */
	dwtStructConfig->chan = c_chNum;
	dwtStructConfig->prf = c_PRF;
	dwtStructConfig->txPreambLength = c_PrmLengthCode;
	dwtStructConfig->rxPAC = c_PACsizeCode;
	dwtStructConfig->rxCode = c_prCode;
	dwtStructConfig->txCode = c_prCode;
	dwtStructConfig->nsSFD = c_nsSFD;   			 // non-standard SFD (for better performance)
	dwtStructConfig->dataRate = c_DataRate;
	dwtStructConfig->phrMode = c_PHR;
	dwtStructConfig->sfdTO = (uint16)DWT_SFDTOC_DEF; // default (longest) SFD timeout
	
}

/**
* @fn getPRMcode
* @param (uint8) c_chNum - Channel number
* @param (uint8) c_PRF 
* @retval Receiver and transmitter preamble code according to Channel number and PRF, as recommended in API Guide.
*/
uint8 getPRMcode(uint8 c_chNum, uint8 c_PRF){
	uint8 c_prCode;
	
	if (c_PRF == DWT_PRF_16M){
		if (c_chNum == 1){
			c_prCode = 1;
		}
		else if (c_chNum == 2 || c_chNum == 5){
			c_prCode = 3;
		}
		else if (c_chNum == 3){
			c_prCode = 5;
		}
		else if (c_chNum == 4 || c_chNum == 7){
			c_prCode = 7;
		}
	}
	else if (c_PRF == DWT_PRF_64M){
		if (c_chNum == 1 || c_chNum == 2 || c_chNum == 3 || c_chNum == 5){
			c_prCode = 9;
		} 
		else if (c_chNum == 4 || c_chNum == 7){
			c_prCode = 17;
		}
	}
	
	return c_prCode;
}

/**
* @fn getPACsize
* @param (uint8) c_PrmLength - Preambule length
* @retval (uint8) c_PACsize - Pac size acorrding to preamble length as is recommended by DW User Manual
*/
uint8 getPACsize(uint8 c_PrmLength){

	uint8 c_PACsize;

	if (c_PrmLength == DWT_PLEN_64 || c_PrmLength == DWT_PLEN_128){
		c_PACsize = DWT_PAC8;
	}
	else if (c_PrmLength == DWT_PLEN_256 || c_PrmLength == DWT_PLEN_512){
		c_PACsize = DWT_PAC16;
	}
	else if (c_PrmLength == DWT_PLEN_1024){
		c_PACsize = DWT_PAC32;
	}
	else if (c_PrmLength == DWT_PLEN_1536 || c_PrmLength == DWT_PLEN_2048 || c_PrmLength == DWT_PLEN_4096){
		c_PACsize = DWT_PAC64;
	}
	
	return c_PACsize;
}

/**
* @fn getPHR
* @param (uint8) c_payload - Payload size
* @retval (uint8) c_PHR - PHR mode acorrding to payload as is recommended by DW User Manual
*/
uint8 getPHR(uint8 c_payload){
	uint8 c_PHR;
	
	if (c_payload < 127){
		c_PHR = DWT_PHRMODE_STD;
	}
	else {
		c_PHR = DWT_PHRMODE_EXT;
	}
	
	return c_PHR;
}

/**
* @fn getPayload
* @param (uint8) c_payloadCode - Payload code
* @retval (uint32_t) w_payloadSize - Payload size according to payload code.
*/
uint32_t getPayload(uint8 c_payloadCode){
	
	uint32_t w_payloadSize;
	
	switch(c_payloadCode){
		case PAYLOAD_10B_CODE:
			w_payloadSize = PAYLOAD_10B;
			break;
		case PAYLOAD_20B_CODE:
			w_payloadSize = PAYLOAD_20B;
			break;
		case PAYLOAD_50B_CODE:
			w_payloadSize = PAYLOAD_50B;
			break;
		case PAYLOAD_100B_CODE:
			w_payloadSize = PAYLOAD_100B;
			break;
		case PAYLOAD_500B_CODE:
			w_payloadSize = PAYLOAD_500B;
			break;
		case PAYLOAD_1000B_CODE:
			w_payloadSize = PAYLOAD_1000B;
			break;
	}
	
	return w_payloadSize;

}

/**
* @fn getDataRateFromCode
* @brief gets used data rate from code
* @param (uint8) c_dataRateCode - Code for data rate
* @retval (uint32_t) w_dataRateData -  rate in kbits/s
*/

uint32_t getDataRateFromCode(uint8 c_dataRateCode){
	
	uint32_t	w_dataRate;
	
	switch (c_dataRateCode){
		case DWT_BR_110K:
			w_dataRate = 110;
			break;
		case DWT_BR_850K:
			w_dataRate = 850;
			break;
		case DWT_BR_6M8:
			w_dataRate = 6800;
			break;
	}
	
	return w_dataRate;
}


/**
* @brief Sends measurement data to host
* @procedure
* 1.  1 byte  - measurement results incoming alert byte
* 2.  1 word  - Number of measuring exchanges (poll - respond)
* 3.  1 word  - time of exchange average ...
* 4.  1 word  - ... and variance
* 5.  1 word  - number of sent messages
* 6.  1 word  - number of received messages
* 7.  1 word  - CIR average ...
* 8.  1 word  - ... and variance
* 9.  1 word  - PRM count average ...
* 10. 1 word  - ... and variance
*/
void sendMeasurementData(void){
	
	/* Compute time of exchange average and variance */
	uint32_t w_measTimValuesAvg = computeAverage(&aw_measTimValues[1], w_recMsgCnt-1, 1);
	uint32_t w_measTimValuesVar = computeVarianceSum(&aw_measTimValues[1], w_recMsgCnt-1, 1);
	
	/* Compute CIR values average and variance */
	uint32_t w_measCIRvaluesAvg = computeAverage(aw_ChImpulseResponse, w_recMsgCnt, 1);
	uint32_t w_measCIRvaluesVar = computeVarianceSum(aw_ChImpulseResponse, w_recMsgCnt, 1);
	
	/* Compute PRM count average and variance */
	uint32_t w_measPRMcntValuesAvg = computeAverage(aw_rxPreamCount, w_recMsgCnt, 1);
	uint32_t w_measPRMcntValuesVar = computeVarianceSum(aw_rxPreamCount, w_recMsgCnt, 1);
	
	/* Compute Tag's turnaroud times average and standard deviation */
	uint32_t w_measTurnarndValuesAvg = computeAverage(&aw_measTurnaroundTimes[1], w_recMsgCnt-1, 1);
	uint32_t w_measTurnarndValuesVar = computeVarianceSum(&aw_measTurnaroundTimes[1], w_recMsgCnt-1, 1);
	
	/* Clear all arrays */
	clearArray(aw_measTimValues, NUM_MEASURE_MESSAGES);
	clearArray(aw_ChImpulseResponse, NUM_MEASURE_MESSAGES);
	clearArray(aw_rxPreamCount, NUM_MEASURE_MESSAGES);
	clearArray(aw_measTurnaroundTimes, NUM_MEASURE_MESSAGES);
	
	/* Send data over USART */
	USART_putc((uint8_t)MEAS_RES_INCOMING);
	USART_putw((uint32_t)NUM_MEASURE_MESSAGES);
	USART_putw(w_measTimValuesAvg);
	USART_putw(w_measTimValuesVar);
	USART_putw(w_trMsgCnt);
	USART_putw(w_recMsgCnt);
	USART_putw(w_measCIRvaluesAvg);
	USART_putw(w_measCIRvaluesVar);
	USART_putw(w_measPRMcntValuesAvg);
	USART_putw(w_measPRMcntValuesVar);
	USART_putw(w_measTurnarndValuesAvg);
	USART_putw(w_measTurnarndValuesVar);
}

void clearArray(uint32_t *aw_array, uint32_t w_size){
	for (uint32_t i = 0; i < w_size; i++ ){
		aw_array[i] = 0;
	}
}

/**
* @brief Computes sum of array
* @param (uint32_t *) aw_wordArray - Input array of words
* @param (uint32_t) w_size - Size of input array
* @retval (uint32_t) w_sum - Computed sum of array
*/
uint32_t computeSum(uint32_t *aw_wordArray, uint32_t w_size){
	
	uint32_t w_sum = 0;
	
	for (uint32_t i = 0; i < w_size; i++){
		w_sum = w_sum + aw_wordArray[i];
	}
	
	return w_sum;
}

/**
* @brief Computes average of array
* @param (uint32_t *)aw_wordArray - Input array of words
* @param (uint32_t)w_size - Size of input array
* @retval (uint32_t) w_avg - Computed average of array
*/
uint32_t computeAverage(uint32_t *aw_wordArray, uint32_t w_size, uint8_t c_excludeZerosEn){
	
	uint32_t w_avg = 0;
	uint32_t w_zerosCnt = 0;
	
	// count zero elements
	if (c_excludeZerosEn){
		for (uint32_t i = 0; i < w_size; i++){
			if (aw_wordArray[i] == 0){
				w_zerosCnt++;
			}
		}
	}
	
	uint32_t w_sum = computeSum(aw_wordArray, w_size);
	
	w_avg = w_sum / (w_size - w_zerosCnt);
	
	return w_avg;
}

/**
* @brief Compute variance of array (non normed). 
* @param (uint32_t *) aw_wordArray - input array of words
* @param (uint32_t) w_size - Size of input array
* @retval (uint32_t) w_var - Computed variance
* @note Result is non-normed and equal to N*v, where N is number of array elements and v is variance.
*/
uint32_t computeVarianceSum(uint32_t *aw_wordArray, uint32_t w_size, uint8_t c_excludeZerosEn){
	
	uint32_t w_avg = computeAverage(aw_wordArray, w_size, c_excludeZerosEn);
	
	int32_t sw_temp[w_size];
	
	/* Compute sumation arguments */
	for (uint32_t i = 0; i < w_size; i++){
		if (c_excludeZerosEn && (aw_wordArray[i] == 0)){
			sw_temp[i] = 0;
		}
		else {
			sw_temp[i] = (int32_t)aw_wordArray[i] - (int32_t)w_avg;
			sw_temp[i] = sw_temp[i] * sw_temp[i]; // square of argument
		}
	}
	
	/* Compute variance */
	uint32_t w_var;
	w_var = computeSum((uint32_t *)sw_temp, w_size);
	
	return w_var;

}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
TIM_HandleTypeDef htim5;

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
static void MX_TIM5_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* DW states for state machine */
uint32_t w_dwtState = DECA_TAG_IDLE_STATE;
uint32_t w_dwtStateLocal = DECA_TAG_IDLE_STATE;

/* Default communication configuration */
static dwt_config_t config = 
	{ // working configuration (this is measurement #200)
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_256,    /* Preamble length. Used in TX only. */
    DWT_PAC16,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_850K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (256 + 1 + 64 - 16) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
	};

/* Measured time of flight in seconds and distance in meters */
static double d_timeOfFlight;
static double d_distance;
	
/* Default antenna delay values for 64 MHz PRF */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* DW module mode*/
/* Set desired operating mode for DW here */
uint32_t w_dwtMode = DWT_MODE_ANCHOR;
uint32_t w_dwtId = DWT_ID_ANCHOR_0;

/* All measured distances from anchors */
uint16_t ahw_distances[NUM_OF_ANCHORS];
	
/* Time of one cycle of measuring all distances*/
uint32_t w_cyclePeriod = 0;
	
/* Time for computing distnace */
uint32_t w_distCompTime = 0;

/* Receive and transmit buffers are set to 20 bytes */
static uint8 ac_rxBuffer[PAYLOAD_20B];
static uint8 ac_txBuffer[PAYLOAD_20B];

/* Index of the current anchor for communication */
uint32_t w_measureAnchorInd = 0;

/* Timestamp fields */
uint8 ac_anchorPoll0rxTS[5];
uint8 ac_tagResponse0rxTS[5];
uint8 ac_tagPoll0txTS[5];
uint8 ac_anchorResponse0txTS[5];
uint8 ac_anchorPoll1rxTS[5];
uint8 ac_tagPoll1txTS[5];

uint64_t dw_anchorPoll0rxTS;
uint64_t dw_tagResponse0rxTS;
uint64_t dw_tagPoll0txTS;
uint64_t dw_anchorResponse0txTS;
uint64_t dw_anchorPoll1rxTS;
uint64_t dw_tagPoll1txTS;

/* Declaration of interrupt callback functions */
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_to_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);
static void tx_conf_cb(const dwt_cb_data_t *cb_data);

/* Function prototypes */
uint64_t getTimestampFromBuffer(uint8_t *ac_buffer, uint32_t w_offset);
void writeTimestampToBuffer(uint8_t *ac_readBuffer, uint8_t *ac_writeBuffer, uint32_t w_offset);

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
  MX_TIM5_Init();

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

	/* Set delay after transmiting */
	//dwt_setrxaftertxdelay(TX_TO_RX_DELAY_UUS);

	/* Apply default antenna delay value */
	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);

	// Check SPI communication by reading ID
	while(dwt_readdevid() != DWT_DEVICE_ID){
		//USART_puts("ID NOT read successfully\n");
		while(1){};
	}
	//USART_puts("ID read successfully\n");
	
	/* Alert host, that MCU reset has occured*/
	//USART_putc(MCU_RESET);
	
	/* Set receive timeout */
	dwt_setrxtimeout(DECA_RX_TIMEOUT); 
	
	/* Set initial state according to selected mode */
	if (w_dwtMode == DWT_MODE_ANCHOR){
		w_dwtState = DECA_ANCHOR_IDLE_STATE;
	}
	else if (w_dwtMode == DWT_MODE_TAG){
		w_dwtState = DECA_TAG_IDLE_STATE;
	}
	
	/* Enable and reset timer */
	TIM2->CR1 |= 0x1; // Timer enable
	TIM2->EGR |= 0x1; // Update event
	
	TIM5->CR1 |= 0x1; // Timer enable
	TIM5->EGR |= 0x1; // Update event
	

	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	
		w_dwtStateLocal = w_dwtState;
		/* Main state machine*/
		switch (w_dwtStateLocal){
			case DECA_TAG_IDLE_STATE:
				
				/* Get index of next anchor for exchange */
				w_measureAnchorInd++;
				if (w_measureAnchorInd >= NUM_OF_ANCHORS){
					w_measureAnchorInd = 0;
				}
				
				/* Send first poll to current Anchor */
				/* Write sender id and message ID into first byte of message*/
				ac_txBuffer[SENDER_INFO_BYTE] =  (uint8_t)(MESSAGE_ID_POLL_0)   |\
																				 (uint8_t)(DWT_MODE_TAG)        |\
																				 (uint8_t)w_dwtId;
				
				/* Write receiver ID to 2nd byte of message */
				ac_txBuffer[RECEIVER_INFO_BYTE] = (uint8_t)(DWT_MODE_ANCHOR)    |\
																					(uint8_t)w_measureAnchorInd;
				
				/* Write distance data to bytes 2 to 7 as 3 half words*/
				for (uint32_t dist_ind = 0; dist_ind < NUM_OF_ANCHORS; dist_ind++){
					for (uint32_t i = 0; i < 2; i++){
						ac_txBuffer[MESSAGE_DISTANCE_BYTE + dist_ind*2 + i] = (uint8_t)(ahw_distances[dist_ind] >> (8*i));
					}
				}
				
				/* Write frame data to DW1000 and prepare transmission. */
				dwt_writetxdata(sizeof(ac_txBuffer), ac_txBuffer, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(ac_txBuffer), 0, 1);				  /* Zero offset in TX buffer,  ranging. */

				/* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
				
				TIM2->EGR |= 0x1; // Update event
				
				w_dwtState = DECA_TAG_RECEIVING_RESPONSE0_STATE;

				break;
				
			case DECA_ANCHOR_IDLE_STATE:
				
				/* Activate reception */
				dwt_rxenable(DWT_START_RX_IMMEDIATE);

				w_dwtState = DECA_ANCHOR_RECEIVING_POLL0_STATE;
			
				TIM2->EGR |= 0x1; // Update event
			
				break;
				
			case DECA_ANCHOR_POLL0_RECEIVED_STATE:
				
				/* Read poll0 received timestamp */
				dwt_readrxtimestamp(ac_anchorPoll0rxTS);
				
				/* Send response 0 message */
				/* Write sender id and message ID into first byte of message*/
				ac_txBuffer[SENDER_INFO_BYTE] =  (uint8_t)(MESSAGE_ID_RESPONSE_0) |\
																				 (uint8_t)(DWT_MODE_ANCHOR)       |\
																				 (uint8_t)w_dwtId;
				
				/* Write receiver ID to 2nd byte of message */
				ac_txBuffer[RECEIVER_INFO_BYTE] = (uint8_t)(DWT_MODE_TAG) |\
																					(uint8_t)DWT_ID_TAG_0;
				
				/* Write frame data to DW1000 and prepare transmission. */
				dwt_writetxdata(sizeof(ac_txBuffer), ac_txBuffer, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(ac_txBuffer), 0, 1);				  /* Zero offset in TX buffer,  ranging. */

				/* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
				
				w_dwtState = DECA_ANCHOR_RECEIVING_POLL1_STATE;
				
				TIM2->EGR |= 0x1; // Tim 2 update event
				
				/* Get period of measuring all distances */
				w_cyclePeriod = TIM5->CNT;
				//w_cyclePeriod = w_cyclePeriod;
				TIM5->EGR |= 0x1; // Tim 5 Update event

				break;
				
			case DECA_TAG_RESPONSE0_RECEIVED_STATE:
				
				/* Get transmit timestamp of poll0 and receive timestamp of response 0*/
				dwt_readtxtimestamp(ac_tagPoll0txTS);
				dwt_readrxtimestamp(ac_tagResponse0rxTS);
			
			
				/* Send Poll 1*/
				/* Write sender id and message ID into first byte of message*/
				ac_txBuffer[SENDER_INFO_BYTE] =  (uint8_t)(MESSAGE_ID_POLL_1)  |\
																				 (uint8_t)(DWT_MODE_TAG)       |\
																				 (uint8_t)w_dwtId;
				
				/* Write receiver ID to 2nd byte of message */
				ac_txBuffer[RECEIVER_INFO_BYTE] = (uint8_t)(DWT_MODE_ANCHOR)   |\
																					(uint8_t)w_measureAnchorInd;
				
				/* Write frame data to DW1000 and prepare transmission. */
				dwt_writetxdata(sizeof(ac_txBuffer), ac_txBuffer, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(ac_txBuffer), 0, 1);				    /* Zero offset in TX buffer,  ranging. */

				/* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
				
				w_dwtState = DECA_TAG_RECEIVING_RESPONSE1_STATE;
				
				TIM2->EGR |= 0x1; // Update event

				break;
				
			case DECA_ANCHOR_POLL1_RECEIVED_STATE:
				
				/* Read poll 1 received timestamp and response 0 transmit timestamp */
				dwt_readrxtimestamp(ac_anchorPoll1rxTS);	
				dwt_readtxtimestamp(ac_anchorResponse0txTS);
				
				/* Send response 1 message */
				/* Write sender id and message ID into first byte of message*/
				ac_txBuffer[SENDER_INFO_BYTE] =  (uint8_t)(MESSAGE_ID_RESPONSE_1) |\
																				 (uint8_t)(DWT_MODE_ANCHOR)       |\
																				 (uint8_t)w_dwtId;
				
				/* Write receiver ID to 2nd byte of message */
				ac_txBuffer[RECEIVER_INFO_BYTE] = (uint8_t)(DWT_MODE_TAG)  |\
																					(uint8_t)DWT_ID_TAG_0;
				
				/* Write all three 40bit timestamps to message */
				writeTimestampToBuffer(ac_anchorPoll0rxTS, ac_txBuffer, MESSAGE_POLL0_RX_TS_BYTE);
				writeTimestampToBuffer(ac_anchorResponse0txTS, ac_txBuffer, MESSAGE_RESPONSE0_TX_TS_BYTE);
				writeTimestampToBuffer(ac_anchorPoll1rxTS, ac_txBuffer, MESSAGE_POLL1_RX_TS_BYTE);
				
				/* Write frame data to DW1000 and prepare transmission. */
				dwt_writetxdata(sizeof(ac_txBuffer), ac_txBuffer, 0);   /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(ac_txBuffer), 0, 1);				    /* Zero offset in TX buffer,  ranging. */

				/* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
				
				w_dwtState = DECA_ANCHOR_IDLE_STATE;
				
				TIM2->EGR |= 0x1; // Update event
				
				break;
				
			case DECA_TAG_RESPONSE1_RECEIVED_STATE:
				
				TIM5->EGR |= 0x1; // update event
				
				/* Get transmit timestamp of poll 1 and receive timestamp of response 1*/
				dwt_readtxtimestamp(ac_tagPoll1txTS);	
				//dwt_readrxtimestamp(ac_tagResponse1rxTS);	
				
				dw_tagPoll0txTS = getTimestampFromBuffer(ac_tagPoll0txTS, 0);
			  dw_tagResponse0rxTS = getTimestampFromBuffer(ac_tagResponse0rxTS, 0);
			  dw_tagPoll1txTS = getTimestampFromBuffer(ac_tagPoll1txTS, 0);

				/* Get all anchor timestamps from response 1 message */
				dw_anchorPoll0rxTS = getTimestampFromBuffer(ac_rxBuffer, MESSAGE_POLL0_RX_TS_BYTE);
				dw_anchorResponse0txTS = getTimestampFromBuffer(ac_rxBuffer, MESSAGE_RESPONSE0_TX_TS_BYTE);
				dw_anchorPoll1rxTS = getTimestampFromBuffer(ac_rxBuffer, MESSAGE_POLL1_RX_TS_BYTE);
				
				/* Compute distance */
				double Ra, Rb, Da, Db;
				int64_t tof_dtu;
				
				Ra = (double)(dw_tagResponse0rxTS - dw_tagPoll0txTS);
				Rb = (double)(dw_anchorPoll1rxTS - dw_anchorResponse0txTS);
				Da = (double)(dw_tagPoll1txTS - dw_tagResponse0rxTS);
				Db = (double)(dw_anchorResponse0txTS - dw_anchorPoll0rxTS);
				
				tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
				
				d_timeOfFlight = tof_dtu * DWT_TIME_UNITS;
				d_distance = d_timeOfFlight * SPEED_OF_LIGHT;
				
				/* Save distance in cm as half word*/
				ahw_distances[w_measureAnchorInd] = (uint16_t)(d_distance * 100); 
				
				w_distCompTime = TIM5->CNT;
				
				w_dwtState = DECA_TAG_IDLE_STATE;
				
				TIM2->EGR |= 0x1; // Update event
				
				break;	
				
			case DECA_ANCHOR_DISTANCES_RECEIVED_STATE:
				/* 
				*	 Tag has sent a poll which is not intended for Anchor 0, 
				*  use this time to read Tag's distance measurement fields and send to host 
				*/
			
				for (uint32_t dist_ind = 0; dist_ind < NUM_OF_ANCHORS; dist_ind++){
					ahw_distances[dist_ind] = 0;
					for (uint32_t i = 0; i < 2; i++){
						ahw_distances[dist_ind] |= ((uint16_t)(ac_rxBuffer[MESSAGE_DISTANCE_BYTE + dist_ind*2 + i]) << (8*i));
					}
				}
				
				
				/* Send measured distances to Host */
				USART_putc(MEAS_INCOMING);
				for (uint32_t i = 0; i < NUM_OF_ANCHORS; i++){
					USART_puthw(ahw_distances[i]);
				}

				w_dwtState = DECA_ANCHOR_IDLE_STATE;
				
				TIM2->EGR |= 0x1; // Update event
				
				break;
				
			case DECA_TAG_RECEIVING_RESPONSE0_STATE:
			case DECA_TAG_RECEIVING_RESPONSE1_STATE:
				
				if (TIM2->CNT > TIM_CNT_TIMEOUT){
					
					/* Force DW transmiter/receiver off */
					dwt_forcetrxoff(); 
					
					w_dwtState = DECA_TAG_IDLE_STATE;
					
					TIM2->EGR |= 0x1; // Update event
				}
				break;
				
			case DECA_ANCHOR_RECEIVING_POLL0_STATE:
			case DECA_ANCHOR_RECEIVING_POLL1_STATE:
					
				if (TIM2->CNT > TIM_CNT_TIMEOUT){
					
					/* Force DW transmiter/receiver off */
					dwt_forcetrxoff(); 
					
					w_dwtState = DECA_ANCHOR_IDLE_STATE;
					
					TIM2->EGR |= 0x1; // Update event	
				}
			
				break;
		}	
	
		
	} /* end while*/
	
  /* USER CODE END 3 */

}

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

/* TIM5 init function */
void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 71;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim5);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);

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
 * @brief Callback to process RX good frame events
 * @param  cb_data  callback data
 * @return  none
 */
static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
	
	int i;
	
	/* Clear local RX buffer to avoid having leftovers from previous receptions. This is not necessary but is included here to aid reading the RX
	 * buffer. */
	for (i = 0 ; i < PAYLOAD_20B; i++ )
	{
		ac_rxBuffer[i] = 0;
	}

	/* Check if received message is expected length */
	if (cb_data->datalength <= PAYLOAD_20B)
	{
		
		/* A frame has been received, copy it to our local buffer. */
		dwt_readrxdata(ac_rxBuffer, cb_data->datalength, 0);
		
		/* Tag */
		if ((w_dwtMode == DWT_MODE_TAG) && (w_dwtId == DWT_ID_TAG_0)){
			
			/* Check message ID */
			switch(ac_rxBuffer[SENDER_INFO_BYTE] & MESSAGE_ID_MASK){
				case MESSAGE_ID_RESPONSE_0:
						if (w_dwtState == DECA_TAG_RECEIVING_RESPONSE0_STATE){
							w_dwtState = DECA_TAG_RESPONSE0_RECEIVED_STATE;
						}
						else {
							w_dwtState = DECA_TAG_IDLE_STATE;
						}
					break;
				case MESSAGE_ID_RESPONSE_1:
					if (w_dwtState == DECA_TAG_RECEIVING_RESPONSE1_STATE){
						w_dwtState = DECA_TAG_RESPONSE1_RECEIVED_STATE;
					}
					else {
						w_dwtState = DECA_TAG_IDLE_STATE;
					}
					break;
				default:
				/* In case message with unknown ID received, cancel exchange and go back to Idle */
				w_dwtState = DECA_TAG_IDLE_STATE;
				}
				
			}
			/* Main anchor */
			else if ((w_dwtMode == DWT_MODE_ANCHOR) && (w_dwtId == DWT_ID_ANCHOR_0)){
				
				/* Check message ID */
				switch (ac_rxBuffer[SENDER_INFO_BYTE] & MESSAGE_ID_MASK){
					case MESSAGE_ID_POLL_0:
						/* Check if message is intended for me*/
						if ((ac_rxBuffer[RECEIVER_INFO_BYTE] & DWT_ID_MASK) == w_dwtId){
							w_dwtState = DECA_ANCHOR_POLL0_RECEIVED_STATE;
						}
						else if ((ac_rxBuffer[RECEIVER_INFO_BYTE] & DWT_ID_MASK) == DWT_ID_ANCHOR_1){
							w_dwtState = DECA_ANCHOR_DISTANCES_RECEIVED_STATE;
						}
						else {
							w_dwtState = DECA_ANCHOR_IDLE_STATE;
						}
						break;
						
					case MESSAGE_ID_POLL_1:
						/* Check if message is intended for me*/
						if ((ac_rxBuffer[RECEIVER_INFO_BYTE] & DWT_ID_MASK) == w_dwtId){
							w_dwtState = DECA_ANCHOR_POLL1_RECEIVED_STATE;
						}
						else {
							w_dwtState = DECA_ANCHOR_IDLE_STATE;
						}
						break;
					default:
						/* In case message with unknown ID received, cancel exchange and go back to Idle */
						w_dwtState = DECA_ANCHOR_IDLE_STATE;
				}
					
			}
			/* All other anchors */
			else {
				
				/* Check message ID */
				switch (ac_rxBuffer[SENDER_INFO_BYTE] & MESSAGE_ID_MASK){
					
					case MESSAGE_ID_POLL_0:
						/* Check if message is intended for me*/
						if ((ac_rxBuffer[RECEIVER_INFO_BYTE] & DWT_ID_MASK) == w_dwtId){
							w_dwtState = DECA_ANCHOR_POLL0_RECEIVED_STATE;
						}
						else{
							w_dwtState = DECA_ANCHOR_IDLE_STATE;
						}
						break;
					
					case MESSAGE_ID_POLL_1:
						/* Check if message is intended for me*/
						if ((ac_rxBuffer[RECEIVER_INFO_BYTE] & DWT_ID_MASK) == w_dwtId){
							w_dwtState = DECA_ANCHOR_POLL1_RECEIVED_STATE;
						}
						else {
							w_dwtState = DECA_ANCHOR_IDLE_STATE;
						}
						break;
						
					default:
						/* In case message with unknown ID received, cancel exchange and go back to Idle */
						w_dwtState = DECA_ANCHOR_IDLE_STATE;
				}
			}

		}
		else {
		/* Incorrect message was received, cancel current exchange and go back to idle state */
		w_dwtState = DECA_ANCHOR_IDLE_STATE;
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
	/* In case timeout occurs during reception, cancel exchange and go back to Idle state */
	switch (w_dwtMode){
		case DWT_MODE_TAG:
			w_dwtState = DECA_TAG_IDLE_STATE;
			break;
		
		case DWT_MODE_ANCHOR:
			w_dwtState = DECA_ANCHOR_IDLE_STATE;	
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
	/* In case error occurs during reception, cancel exchange and go back to Idle state */
	switch (w_dwtMode){
		case DWT_MODE_TAG:
			w_dwtState = DECA_TAG_IDLE_STATE;
			break;
		
		case DWT_MODE_ANCHOR:
			w_dwtState = DECA_ANCHOR_IDLE_STATE;	
			break;
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tx_conf_cb()
 * @brief Callback to process TX confirmation events
 * @param  cb_data  callback data
 * @return  none
 */
static void tx_conf_cb(const dwt_cb_data_t *cb_data)
{

}

/**
 * @fn getTimestampFromBuffer
 * @brief Get timestamp from buffer
 * @param  (uint8_t *) ac_buffer   : buffer from which the timestamp will be read
 * @param  (uint32_t) w_offset     : offset from buffer base address
 * @return (uint64_t) dw_timestamp : 5 byte timestamp value
 */
uint64_t getTimestampFromBuffer(uint8_t *ac_buffer, uint32_t w_offset){
	
	uint64_t dw_timestamp = 0;
	for (uint32_t i = 0; i < 5; i++) {
		dw_timestamp |= (uint64_t)(ac_buffer[w_offset + i]) << (8*i);
	}
	
	return dw_timestamp;
	
}
/**
 * @fn writeTimestampToBuffer
 * @brief Write 5 byte timestamp to buffer
 * @param  (uint64_t) ac_readBuffer : read buffer from which 5 byte timestamp value will be read
 * @param  (uint8_t *) ac_buffer    : buffer to which the timestamp will be written
 * @param  (uint32_t) w_offset      : offset from write buffer base address
 * @return none
 */
void writeTimestampToBuffer(uint8_t *ac_readBuffer, uint8_t *ac_writeBuffer, uint32_t w_offset){
		
	for (uint32_t i = 0; i < 5; i++){
		ac_writeBuffer[w_offset + i] = ac_readBuffer[i];
	}
	
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

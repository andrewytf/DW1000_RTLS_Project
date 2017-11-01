/* Message types definitions */
#define MSG_TYPE_CONF_SET   	    0x01
#define MSG_TYPE_CONF_CONFIRM     0x02
#define MSG_TYPE_MEAS_POLL   	    0x03
#define MSG_TYPE_MEAS_RESPONSE   	0x04
#define MSG_TYPE_MEAS_END   			0x05

/* Messages length definitions */
#define CONFIG_MSG_LENGTH  10

/* Payloads definitions */
#define PAYLOAD_10B      10
#define PAYLOAD_20B			 20
#define PAYLOAD_50B			 50
#define PAYLOAD_100B	  100
#define PAYLOAD_500B    500
#define PAYLOAD_1000B	 1000

/* Payload code definitions */
#define PAYLOAD_10B_CODE       1
#define PAYLOAD_20B_CODE			 2
#define PAYLOAD_50B_CODE			 3
#define PAYLOAD_100B_CODE	     4
#define PAYLOAD_500B_CODE      5
#define PAYLOAD_1000B_CODE	   6

/* Configuration message byte definitions*/
#define MSG_TYPE_BYTE 			   0
#define CHANNEL_NUMBER_BYTE    1
#define PRF_BYTE 						   2
#define DATA_RATE_BYTE 			   3
#define PRM_LENGTH_BYTE 			 4
#define PAYLOAD_SIZE_BYTE 		 5

/* Poll message byte definitions*/
#define TIMESTAMP_OFFSET       1
#define POLL_NUM_BYTE          5

/* Deca states definitions*/
#define DECA_STATE_IDLE  								     0x00
#define DECA_ANCHOR_STATE_IDLE					     0x01
#define DECA_TAG_CONFIG_RECEIVED_STATE	     0x10		
#define DECA_ANCHOR_CONFIRM_RECEIVED_STATE   0x11
#define DECA_TAG_POLL_RECEIVED_STATE         0x12
#define DECA_ANCHOR_RESPONSE_RECEIVED_STATE  0x13
#define DECA_UNKNOWN_RECEIVED_STATE          0x14
#define DECA_TAG_END_RECEIVED_STATE          0x15
#define DECA_ANCHOR_SENDING_CONFIG_STATE		 0x16
#define DECA_STATE_RECEIVING_RESPONSE        0x20
#define DECA_ANCHOR_RECEIVING_RESPONSE_STATE 0x21
#define DECA_ANCHOR_RECEIVING_CONFIRM_STATE  0x22
#define DECA_TAG_RECEIVING_STATE						 0x23
#define DECA_TAG_RECEIVING_POLL_STATE        0x40
#define DECA_TAG_RECEIVING_CONFIG_STATE			 0x41 
#define DECA_ANCHOR_REC_CONFIRM_TMOUT_STATE  0x50
#define DECA_ANCHOR_REC_RESPONSE_TMOUT_STATE 0x51
#define DECA_TAG_REC_POLL_TMOUT_STATE        0x53
#define DECA_TAG_REC_CONFIG_TMOUT_STATE      0x54
#define DECA_TAG_RECEIVING_TMOUT_STATE       0x55

/* Number of messages to exchange for one measurement*/
#define NUM_MEASURE_MESSAGES 101

#define RX_RESP_TIMEOUT_CNT_MAX    30
#define RX_CONF_TIMEOUT_CNT_MAX    100
#define RX_POLL_TIMEOUT_CNT_MAX		 30
#define RX_ERR_CNT_MAX  					 10

/* USART codes */
#define NEW_CONFIGURATION_INCOMING   0x01
#define USART_NUM_OF_CONFIG 				    5

/* Timeout values */
#define DWT_ANCHOR_RECEIVE_TIMEOUT  50000
#define DWT_TAG_RECEIVE_TIMEOUT     50000
#define TIM_CNT_TIMEOUT           5000000
#define TIM_TAG_CNT_TIMEOUT    RX_POLL_TIMEOUT_CNT_MAX * DWT_TAG_RECEIVE_TIMEOUT
#define RX_TIMEOUT_OFFSET  				10000

#define TX_TO_RX_DELAY_UUS 60

/* Response codes for host */
#define MEASUREMENT_CPLT 					 0x55
#define MEAS_RES_INCOMING				   0x44
#define MEAS_RES_END 						 	 0x33
#define MEAS_ERROR_CONFIG          0x22
#define MEAS_ERROR_RESP  				   0x11
#define MEAS_RX_ERROR     				 0x66
#define MEAS_UNKNOWN_REC_ERROR     0x68
#define MCU_RESET    							 0x77

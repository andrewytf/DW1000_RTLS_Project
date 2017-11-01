/* Payloads definitions */
#define PAYLOAD_20B			 20

#define DECA_RX_TIMEOUT 800

/* DW states definitions*/
#define DECA_TAG_IDLE_STATE                  0x00
#define DECA_TAG_RECEIVING_RESPONSE0_STATE   0x01
#define DECA_TAG_RESPONSE0_RECEIVED_STATE    0x02
#define DECA_TAG_RECEIVING_RESPONSE1_STATE   0x03
#define DECA_TAG_RESPONSE1_RECEIVED_STATE    0x04
#define DECA_ANCHOR_IDLE_STATE							 0x20
#define DECA_ANCHOR_RECEIVING_POLL0_STATE    0x21
#define DECA_ANCHOR_POLL0_RECEIVED_STATE     0x22
#define DECA_ANCHOR_RECEIVING_POLL1_STATE    0x23
#define DECA_ANCHOR_POLL1_RECEIVED_STATE     0x24
#define DECA_ANCHOR_DISTANCES_RECEIVED_STATE 0x30

#define TX_TO_RX_DELAY_UUS 60

/* Number of used anchors in RTLS*/
#define NUM_OF_ANCHORS 0x3

/********* Message byte definitions *********/

/**
* Frame is 20 bytes long.
* [0]     -> Transmiter and message information
* [1]     -> Receiver information
* [2...7] -> Three 2 byte measured distances (Tag mode)
* [2..16] -> Three 5 byte timestamps (Anchor mode)
*/

#define SENDER_INFO_BYTE   0x0
#define RECEIVER_INFO_BYTE 0x1

/* DW mode byte definitions */
#define DWT_MODE_MASK     0x08 // 0b1000
#define DWT_MODE_OFFSET   0x03
#define DWT_MODE_ANCHOR   0x08
#define DWT_MODE_TAG      0x00

/* DW ID byte definitions */
#define DWT_ID_MASK       0x07 // 0b0111
#define DWT_ID_OFFSET     0x00
#define DWT_ID_TAG_0      0x00 
#define DWT_ID_ANCHOR_0   0x00
#define DWT_ID_ANCHOR_1   0x01
#define DWT_ID_ANCHOR_2   0x02

/* Message ID byte definitions */
#define MESSAGE_ID_MASK        0xF0
#define MESSAGE_ID_OFFSET			 0x4
#define MESSAGE_ID_POLL_0      0x00
#define MESSAGE_ID_RESPONSE_0  0x10
#define MESSAGE_ID_POLL_1      0x20
#define MESSAGE_ID_RESPONSE_1  0x30

/* Distance byte definitons */
#define MESSAGE_DISTANCE_BYTE 0x2

/* Timestamps byte definitions */
#define MESSAGE_POLL0_RX_TS_BYTE      0x02
#define MESSAGE_RESPONSE0_TX_TS_BYTE  0x07
#define MESSAGE_POLL1_RX_TS_BYTE      0x0C
/********************************************/

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* USART codes definitions */
#define MEAS_INCOMING    0x55

#define TIM_CNT_TIMEOUT 5000


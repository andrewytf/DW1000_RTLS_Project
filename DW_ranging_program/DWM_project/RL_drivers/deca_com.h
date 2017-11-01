#include "stm32l4xx_hal.h"

/* Function Prototypes */
void reset_DW1000(void);
void wakeup_DW1000(void);

#define DECA_MODE_TAG    0x0
#define DECA_MODE_ANCHOR 0x1

#define DECA_TEST_TRANSMIT 0x0
#define DECA_TEST_RECEIVE  0x1



typedef enum {
	
	DECA_INIT,
	DECA_TXBLINK_WAIT_SEND,
	DECA_TXPOLL_WAIT_SEND,
	DECA_RXE_WAIT,
	DECA_RX_WAIT_DATA,
	DECA_SLEEP_DONE,
	DECA_TXE_WAIT,
	DECA_TXFINAL_WAIT_SEND,
	DECA_TX_WAIT_CONF

} app_deca_state;


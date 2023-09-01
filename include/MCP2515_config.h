/******************************************************/
/******************************************************/
/**********      AUTHOR: Raghad Mohamed      **********/
/**********      Layer: HAL                  **********/
/**********      SWC: MCP2515                **********/
/**********      Date: 28-10-2021            **********/
/**********      Version: 1.00               **********/
/******************************************************/
/******************************************************/



#ifndef MCP2515_CONFIG_H_
#define MCP2515_CONFIG_H_

#include "MCP2515_private.h"

/* Used to select the mode of operation of the MCP2515
 * Options: 1. NORMAL_MODE
 *          2. SLEEP_MODE
 *          3. LOOPBACK_MODE 
 *          4. LISTEN_ONLY_MODE
 *          5. CONFIGURATION_MODE
 */         
#define MODE_OF_OPERATION   NORMAL_MODE


/* Select which board this driver operates on currently
 * Options: 1. MCP_COMMUNICATION_BOARD
 *					2. MCP_FEEDBACK_BOARD
 */	
#define BOARD_ID		MCP_COMMUNICATION_BOARD
//#define BOARD_ID		MCP_FEEDBACK_BOARD
 


#define CS_PORT			GPIOA  			/* Select the port of the CS (Chip Select) Pin */
#define CS_PIN 			GPIO_PIN_4  /* Select the pin of the CS (Chip Select) Pin */





#endif

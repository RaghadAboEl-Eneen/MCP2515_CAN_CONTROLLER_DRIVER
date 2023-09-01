/******************************************************/
/******************************************************/
/**********      AUTHOR: Raghad Mohamed      **********/
/**********      Layer: HAL                  **********/
/**********      SWC: MCP2515                **********/
/**********      Date: 28-10-2021            **********/
/**********      Version: 1.00               **********/
/******************************************************/
/******************************************************/

#ifndef MCP2515_PRIVATE_H_
#define MCP2515_PRIVATE_H_

#define MCP_COMMAND_RESET           0xC0		/* Resets internal registers to the default state, sets configuration mode */

#define MCP_COMMAND_READ            0x03		/* Read data from the register beginning at selected register */


#define MCP_COMMAND_READ_RX_BUFFER_ID   0x90
#define MCP_COMMAND_READ_RX_BUFFER_DATA 0x92			 /* Read receive buffer	
																										* 0b10010nm0 << nm: place the address pointer at nm
																										*/

#define MCP_COMMAND_WRITE           0x02		/* Writes data to the selected register beginning at the selected address */



#define MCP_COMMAND_LOAD_TX_BUFFER_ID  			0x40    
#define MCP_COMMAND_LOAD_TX_BUFFER_DATA			0x41    /* Load transmit buffer 
																										 * 0b010000abc << abc: place the address pointer at abc
																										 */


#define MCP_COMMAND_RTS             0x81	/* Message Request To Send
																					 * Instructs controller to begin message transmission 
																					 * sequence for any of the transmit buffers
																					 * 0b10000nnn << nnn: first n:  request to send for TXB2 
																					 *  									middle n: request to send for TXB1
																				   *                    last n:   request to send for TXB0
																					 */

#define MCP_COMMAND_READ_STATUS     0xA0  /* Quick polling command that reads several status bits for 
																					 * transmit and recieve functions 
																					 */

#define MCP_COMMAND_RX_STATUS       0xB0  /* Quick polling command that indicates filter match 
																					 * and message type(standard, extended and/or remote) of received message
																					 */

#define MCP_COMMAND_BIT_MODIFY      0x05  /* Allows the user to set or clear individual bits in a particular register */


#define MCP_COMMUNICATION_BOARD				0x30	/* ID of boards */
#define MCP_FEEDBACK_BOARD					0x40


#define ONE_BIT_MASK								0x01//0b00000001
#define TWO_BIT_MASK								0x03//0b00000011
#define THREE_BIT_MASK							0x07//0b00000111
#define FOUR_BIT_MASK								0x0F//0b00001111
#define FIVE_BIT_MASK								0x1F//0b00011111
#define SIX_BIT_MASK								0x3F//0b00111111





#define MODE_OF_OPERATION_MASK			0xE0  /* 0b1110 0000 */
#define PROPSEG_MASK								0x07  /* 0b0000 0111 */
#define PS1_MASK										0x38  /* 0b0011 1000 */
#define PS2_MASK										0x07  /* 0b0000 0111 */
#define BTLMODE_MASK								0x80	/* 0b1000 0000 */



#define TQ_PROPSEG									0x01  /* 2 TQs */  /* CNF2 */
#define TQ_PS1											0x02  /* 3 TQs */	 /* CNF2 */
#define TQ_PS2  										0x01  /* 2 TQs */  /* CNF3 */
#define BTLMODE_VALUE 							0x01  

#endif

/******************************************************/
/******************************************************/
/**********      AUTHOR: Raghad Mohamed      **********/
/**********      Layer: HAL                  **********/
/**********      SWC: MCP2515                **********/
/**********      Date: 28-10-2021            **********/
/**********      Version: 1.00               **********/
/******************************************************/
/******************************************************/



#ifndef MCP2515_REGISTERS_H_
#define MCP2515_REGISTERS_H_


#define CANSTAT             0x0E    	/* CAN Control Register */
#define CANSTAT_OPMOD2      7       	/* Operation Mode Bit 2 */
#define CANSTAT_OPMOD1      6       	/* Operation Mode Bit 1 */
#define CANSTAT_OPMOD0      5       	/* Operation Mode Bit 0 */
#define CANSTAT_ICOD2       3       	/* Interrupt Flag Code Bit 2 */
#define CANSTAT_ICOD1       2       	/* Interrupt Flag Code Bit 1 */
#define CANSTAT_ICOD0       1       	/* Interrupt Flag Code Bit 0 */


#define CANCTRL             0x0F    	/* CAN Status Register */
#define CANCTRL_REQOP2      7       	/* Request Operation Mode Bit 2 */
#define CANCTRL_REQOP1      6       	/* Request Operation Mode Bit 1 */
#define CANCTRL_REQOP0      5       	/* Request Operation Mode Bit 0 */
#define CANCTRL_ABAT        4       	/* Abort All Pending Transmissions Bit */
#define CANCTRL_OSM         3       	/* One Shot Mode Bit */
#define CANCTRL_CLKEN       2       	/* CLKOUT Pin Enable Bit */
#define CANCTRL_CLKPRE1     1       	/* CLKOUT Pin Prescalar Bit 1 */   
#define CANCTRL_CLKPRE0     0       	/* CLKOUT Pin Prescalar Bit 0 */


#define TXB0CTRL            0x30    	/* Transmit Buffer 0 Control Register */
#define TXB0CTRL_ABTF       6       	/* Message Aborted Flag Bit */
#define TXB0CTRL_MLOA       5       	/* Message Lost Arbitration Bit */
#define TXB0CTRL_TXERR      4       	/* Transmission Error Detected Bit */
#define TXB0CTRL_TXREQ      3       	/* Message Transmit Request Bit  */
#define TXB0CTRL_TXP1       1       	/* Transmit Buffer Prioirty Bit 1 */
#define TXB0CTRL_TXP0       0       	/* Transmit Buffer Priority Bit 0 */


#define TXRTSCTRL           0x0D    	/* TX0RTS Pin Control and Status Register */
#define TXRTSCTRL_B0RTS     3       	/* TX0RTS Pin State Bit */
#define TXRTSCTRL_B0RTSM    0       	/* Tx0RTS Pin Mode bit */


#define TXB0SIDH            0x31    	/* Transmit BUffer 0 Standard Identifier Register High */
#define TXB0SIDL            0x32    	/* Transmit Buffer 0 Standard Identifier Register Low */


#define TXB0DLC             0x35    	/* Transmit Buffer 0 Data Length Code Register */
#define TXB0DLC_RTR         6       	/* Remote Transmission Request Bit */
#define TXB0DLC_DLC3        3       	/* Data Length Code Bit 3 */ 
#define TXB0DLC_DLC2        2       	/* Data Length Code Bit 2 */
#define TXB0DLC_DLC1        1       	/* Data Length Code Bit 1 */
#define TXB0DLC_DLC0        0       	/* Data Length Code Bit 0 */


#define TXB0D0              0x36    	/* Transmit Buffer 0 Data Byte 0 Register */
#define TXB0D1              0x37    	/* Transmit Buffer 0 Data Byte 0 Register */
#define TXB0D2              0x38      /* Transmit Buffer 0 Data Byte 0 Register */
#define TXB0D3              0x39      /* Transmit Buffer 0 Data Byte 0 Register */
#define TXB0D4              0x3A      /* Transmit Buffer 0 Data Byte 0 Register */
#define TXB0D5              0x3B      /* Transmit Buffer 0 Data Byte 0 Register */
#define TXB0D6              0x3C      /* Transmit Buffer 0 Data Byte 0 Register */
#define TXB0D7              0x3D      /* Transmit Buffer 0 Data Byte 0 Register */


#define RXB0CTRL            0x60      /* Receive Buffer 0 Control Register */
#define RXB0CTRL_RXM1       6         /* Receive Buffer Operating Mode Bit 1 */
#define RXB0CTRL_RXM0       5         /* Receive Buffer Operating Mode Bit 0  */
#define RXB0CTRL_RXRTR      3         /* Received Remote Trasnfer Request Bit  */
#define RXB0CTRL_BUKT       2         /* Rollover Enable Bit */
#define RXB0CTRL_FILHIT0    0         /* Filter Hit Bit */


#define BFPCTRL             0x0C      /* RxnBF Pin Control and Status Register */
#define BFPCTRL_B0BFS       4         /* RX0BF Pin State Bit (Digital Output mode only) */
#define BFPCTRL_B0BFE       2         /* RX0BF Pin Function Enable Bit */
#define BFPCTRL_B0BFM       0         /* RX0BF Pin Operation Mode Bit */


#define RXB0SIDH            0x61      /* Receive Buffer 0 Standard Identifier Register High */
#define RXB0SIDL            0x62      /* Receive Buffer 0 Standard Identifier Register Low */


#define RXB0DLC             0x65      /* Receive Buffer 0 Data Length Code Register */
#define RXB0DLC_DLC3        3         /* Data Length Code Bit 3 */
#define RXB0DLC_DLC2        2         /* Data Length Code Bit 2 */
#define RXB0DLC_DLC1        1         /* Data Length Code Bit 1 */
#define RXB0DLC_DLC0        0         /* Data Length Code Bit 0  */


#define RXB0D0              0x66      /* Receive Buffer 0 Data Byte 0 Register */
#define RXB0D1              0x67      /* Receive Buffer 0 Data Byte 1 Register */
#define RXB0D2              0x68      /* Receive Buffer 0 Data Byte 2 Register */
#define RXB0D3              0x69      /* Receive Buffer 0 Data Byte 3 Register */    
#define RXB0D4              0x6A      /* Receive Buffer 0 Data Byte 4 Register */
#define RXB0D5              0x6B      /* Receive Buffer 0 Data Byte 5 Register */
#define RXB0D6              0x6C      /* Receive Buffer 0 Data Byte 6 Register */
#define RXB0D7              0x6D      /* Receive Buffer 0 Data Byte 7 Register */


#define CNF1								0x2A
#define CNF1_SJW1						7
#define CNF1_SJW0						6
#define CNF1_BPR5						5
#define CNF1_BPR4						4
#define CNF1_BPR3						3
#define CNF1_BPR2						2
#define CNF1_BPR1						1
#define CNF1_BPR0						0



#define CNF2								0x29
#define CNF2_BTLMODE				7
#define CNF2_SAM						6
#define CNF2_PHSEG1_2				5
#define CNF2_PHSEG1_1				4
#define CNF2_PHSEG1_0				3
#define CNF2_PRSEG2					2
#define CNF2_PRSEG1					1
#define CNF2_PRSEG0					0



#define CNF3								0x28
#define CNF3_SOF						7
#define CNF3_WAKFIL					6
#define CNF3_PHSEG2_2				2
#define CNF3_PHSEG2_1				1
#define CNF3_PHSEG2_0				0


#define CANINTF							0x2C
#define CANINTF_RX0IF				0
#define CANINTF_RX1IF				1
#define CANINTF_TX0IF				2
#define CANINTF_TX1IF				3	
#define CANINTF_TX2IF				4



#endif

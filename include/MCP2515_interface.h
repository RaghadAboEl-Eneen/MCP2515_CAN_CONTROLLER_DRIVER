/******************************************************/
/******************************************************/
/**********      AUTHOR: Raghad Mohamed      **********/
/**********      Layer: HAL                  **********/
/**********      SWC: MCP2515                **********/
/**********      Date: 28-10-2021            **********/
/**********      Version: 1.00               **********/
/******************************************************/
/******************************************************/


#ifndef MCP2515_INTERFACE_H_
#define MCP2515_INTERFACE_H_

#define NORMAL_MODE         0
#define SLEEP_MODE          1
#define LOOPBACK_MODE       2
#define LISTEN_ONLY_MODE    3
#define CONFIGURATION_MODE  4

#define TRANSMIT_ID					5	
#define TRANSMIT_DATA				6

#define NOOK								7
#define OK									8

#define MCP_ERROR_MISCOMMUNICATION 0xEE



void delay_us(uint32_t delay_us);

void MCP2515_voidBitModify(uint8_t Copy_uint8_tBaseAddress, uint8_t * Copy_puint8_tData, uint8_t * Copy_puint8_tMask); //done

void MCP2515_voidInitiateTransmission(void); //done

void MCP2515_uint8_tReadStatusByte(uint8_t * Copy_puint8_tReceivedData); //done

void MCP2515_uint8_tReadReceiveBuffer(uint8_t * Copy_puint8_tReceivedData, uint8_t Copy_uint8_tType); //done

void MCP2515_voidLoadTransmitBuffer(uint8_t * Copy_puint8_tData, uint8_t Copy_uint8_tType); //done

void MCP2515_voidReset(void); //done

void MCP2515_voidRead(uint8_t Copy_uint8_tBaseAddress, uint8_t * Copy_puint8_tReceivedData); //done

void MCP2515_voidWrite(uint8_t Copy_uint8_tBaseAddress, uint8_t * Copy_puint8_tData);//done

void MCP2515_voidSetOperationMode(uint8_t Copy_u8OperationMode);

void MCP2515_voidInit(void); //done

void MCP2515_voidSetBitRate(void); //done

void MCP2515_voidSendDataByte(uint8_t Copy_uint8_tDataByte, uint8_t Copy_uint8_tID);

uint8_t MCP2515_voidReceiveDataByte(uint8_t * Copy_puint8_tReceivedData,
									uint8_t Copy_uint8_tID,
									uint64_t Copy_uint64timeout);

void MCP2515_voidSetSPI1(SPI_HandleTypeDef hspi);

void MCP2515_voidSetSPI2(SPI_HandleTypeDef hspi);

void MCP2515_voidSetUART1(UART_HandleTypeDef huart);

void MCP2515_voidSetUART2(UART_HandleTypeDef huart);


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
 
#define CS_PORT			GPIOA  			/* Select the port of the CS (Chip Select) Pin */
#define CS_PIN 			GPIO_PIN_4  /* Select the pin of the CS (Chip Select) Pin */


#endif

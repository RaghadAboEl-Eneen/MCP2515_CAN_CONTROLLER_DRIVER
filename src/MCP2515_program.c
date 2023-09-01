/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "MCP2515_config.h"
#include "MCP2515_private.h"
#include "MCP2515_registers.h"
#include "MCP2515_interface.h"




/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi_1;
SPI_HandleTypeDef hspi_2;

UART_HandleTypeDef huart_1;
UART_HandleTypeDef huart_2;

/* USER CODE BEGIN PV */

uint8_t MCP_uint8_tData = 0;
uint8_t MCP_uint8_tMask = 0;


void MCP2515_voidSetSPI1(SPI_HandleTypeDef hspi) {

	hspi_1 = hspi;

}

void MCP2515_voidSetSPI2(SPI_HandleTypeDef hspi) {

	hspi_2 = hspi;

}


void MCP2515_voidSetUART1(UART_HandleTypeDef huart) {

	huart_1 = huart;

}

void MCP2515_voidSetUART2(UART_HandleTypeDef huart) {

	huart_2 = huart;

}



void MCP2515_voidReset(void) {
		
	/* We load the command buffer with the needed command */
	uint8_t Local_uint8_tCommandBuffer = MCP_COMMAND_RESET;
	
	/* First we drive the CS pin low */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);

	/* Then we send the instruction packet MCP_COMMAND_RESET via SPI1 */
	HAL_SPI_Transmit(&hspi_1, &Local_uint8_tCommandBuffer, 1, HAL_MAX_DELAY);
	
	/* After packet is sent, CS pin is driven high once again */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	
}

/**
  * @brief  Writes to the specified base address the data given.
  * 			  Note that if we send more than 1 byte of data
	* 				the base address will automatically get incremeneted
	* 				and data will be written to sequential addresses
	* 				up to 8 bytes of data
  * @param  Copy_uint8_tBaseAddress: Address we want to write to.
	*				  To see full list of registers please view MCP2515_registers.h
  * @param  Copy_puint8_tData: Pointer to the data, can have a maximum size of 8 bytes only.
  * @retval None
  */
void MCP2515_voidWrite(uint8_t Copy_uint8_tBaseAddress, uint8_t * Copy_puint8_tData) {
	
	/* We load the command buffer with the needed command */
	uint8_t Local_uint8_tCommandBuffer = MCP_COMMAND_WRITE;
	
	/* We load the base address buffer with the given base address*/
	uint8_t Local_uint8_tBaseAddressBuffer = Copy_uint8_tBaseAddress;
	
	/* First we drive the CS pin low */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	
	/* We send the instruction packet MCP_COMMAND_WRITE via SPI1 */
	HAL_SPI_Transmit(&hspi_1, &Local_uint8_tCommandBuffer, 1, HAL_MAX_DELAY);

	/* Then we send the address of the register we want to write to*/
	HAL_SPI_Transmit(&hspi_1, &Local_uint8_tBaseAddressBuffer, 1, HAL_MAX_DELAY);
	
	/* Then we send the data we wish to write to the address */
	HAL_SPI_Transmit(&hspi_1, Copy_puint8_tData, 1, HAL_MAX_DELAY);
	
  /* After packet is sent, CS pin is driven high once again */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);	

}


void MCP2515_voidRead(uint8_t Copy_uint8_tBaseAddress, uint8_t * Copy_puint8_tReceivedData) {
	
		/* We load the command buffer with the needed command MCP_COMMAND_READ */
	  uint8_t Local_uint8_tCommandBuffer = MCP_COMMAND_READ;
		
		/* We load the base address buffer with the given base address*/
		uint8_t Local_uint8_tBaseAddressBuffer = Copy_uint8_tBaseAddress;
		
		/* First we drive the CS pin low */
		HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
		
		/* We send the instruction packet MCP_COMMAND_READ via SPI1 */
		HAL_SPI_Transmit(&hspi_1, &Local_uint8_tCommandBuffer, 1, HAL_MAX_DELAY);
		
		/* We send the address of the register we want to read */
		HAL_SPI_Transmit(&hspi_1, &Local_uint8_tBaseAddressBuffer, 1, HAL_MAX_DELAY);
	   
		/* We then receive the data sent over by the MCP via SPI1 */
		HAL_SPI_Receive(&hspi_1, Copy_puint8_tReceivedData,1 , HAL_MAX_DELAY);
		
		/* CS pin is driven high once again to stop communication */
	  HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);		
	
}

/**
  * @brief  The READ_STATUS instruction allows single instruction access to some of the
  * 			  often used status bits for message reception and transmission
	*				  
  * @param  Copy_puint8_tData: Pointer to the received data.
	*					Will hold the status byte.
  * @retval None
  */
void MCP2515_uint8_tReadStatusByte(uint8_t * Copy_puint8_tReceivedData) {
	
		
	/* We load the command buffer with the needed command MCP_COMMAND_READ_STATUS */
	uint8_t Local_uint8_tCommandBuffer[] = {MCP_COMMAND_READ_STATUS};
	
	/* First we drive the CS pin low */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	
	/* We send the instruction packet MCP_COMMAND_READ_STATUS via SPI1 */
	HAL_SPI_Transmit(&hspi_1, Local_uint8_tCommandBuffer, 1, HAL_MAX_DELAY);
	
	/* We then receive the dats sent over by the MCP via SPI1 */
	HAL_SPI_Receive(&hspi_1, Copy_puint8_tReceivedData, 1, HAL_MAX_DELAY);
	   
	/* CS pin is driven high once again to stop communication */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	
	
}


 /*
  * @brief  The RTS (request to send) instruction can be used to initiate message transmission for 
  * 			  one or more transmit buffers
	*				  
  * @param  None
	*					
  * @retval None
  */
void MCP2515_voidInitiateTransmission(void) {
	
	/* We load the command buffer with the needed command MCP_COMMAND_RTS*/
	uint8_t Local_uint8_tCommandBuffer = MCP_COMMAND_RTS;
	
	uint8_t Local_uint8_tData = 0x01;
	uint8_t Local_uint8_tMask = 0x0F;
	
	
	MCP2515_voidBitModify(TXB0DLC,&Local_uint8_tData, &Local_uint8_tMask);
	
	/* First we drive the CS pin low */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	
	/* We send the instruction packet MCP_COMMAND_RTS via SPI1 
	 * This command sets the TXREQ bit (TXBnCTRL[3])
	 */
	HAL_SPI_Transmit(&hspi_1, &Local_uint8_tCommandBuffer, 1, HAL_MAX_DELAY);
	
	/* CS pin is driven high once again to stop communication */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	
		
}



void MCP2515_voidBitModify(uint8_t Copy_uint8_tBaseAddress, uint8_t * Copy_puint8_tData, uint8_t * Copy_puint8_tMask) {
	
	/* We load the command buffer with the needed command MCP_COMMAND_BIT_MODIFY*/
	uint8_t Local_uint8_tCommandBuffer = MCP_COMMAND_BIT_MODIFY;
	
	/* We load the base address buffer with the needed address*/
	uint8_t Local_uint8_tBaseAddressBuffer = Copy_uint8_tBaseAddress;

	/* First we drive the CS pin low */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	
	/* We send the instruction packet MCP_COMMAND_BIT_MODIFY via SPI1 */
	HAL_SPI_Transmit(&hspi_1, &Local_uint8_tCommandBuffer, 1, HAL_MAX_DELAY);
	
	/* Then we send the address of the register we want to write to*/
	HAL_SPI_Transmit(&hspi_1, &Local_uint8_tBaseAddressBuffer, 1, HAL_MAX_DELAY);
	
	/* Then we send the mask of the data */
	HAL_SPI_Transmit(&hspi_1, Copy_puint8_tMask, 1, HAL_MAX_DELAY);
	
	/* Then we send the data itself */
	HAL_SPI_Transmit(&hspi_1, Copy_puint8_tData, 1, HAL_MAX_DELAY);

	/* CS pin is driven high once again to stop communication */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	
}




 /*
  * @brief  The READ_RX_BUFFER command works similarly to the READ command
  * 			  The associated receive flag is cleared automatically
	*				  Sequential reads to the buffers is possible but currently
  *         not implemented.
  * @param  None
	*					
  * @retval None
  */


void MCP2515_uint8_tReadReceiveBuffer(uint8_t * Copy_puint8_tReceivedData, uint8_t Copy_uint8_tType) {
	
	
	uint8_t Local_uint8_tCommandBuffer;
	
	///* We load the command buffer with the needed command MCP_COMMAND_READ_RX_BUFFER*/
	if(Copy_uint8_tType == TRANSMIT_ID) 
		 Local_uint8_tCommandBuffer = MCP_COMMAND_READ_RX_BUFFER_ID;
	else if(Copy_uint8_tType == TRANSMIT_DATA) 
		 Local_uint8_tCommandBuffer = MCP_COMMAND_READ_RX_BUFFER_DATA;
	
	/* First we drive the CS pin low */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	
	/* We send the instruction packet MCP_COMMAND_READ_RX_BUFFER via SPI1 */
	HAL_SPI_Transmit(&hspi_1, &Local_uint8_tCommandBuffer, 1, HAL_MAX_DELAY);
	
	/* We then receive the dats sent over by the MCP via SPI1 */
	HAL_SPI_Receive(&hspi_1, Copy_puint8_tReceivedData, 1, HAL_MAX_DELAY);
	
	/* CS pin is driven high once again to stop communication */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	
	
}



 /*
  * @brief  The LOAD_TX_BUFFER command quickly writes to the transmit buffer 
  *         that points to the ID or DATA address of any of the three transmit buffers. 
  * 			  
	*				  
  * @param  None
	*					
  * @retval None
  */
void MCP2515_voidLoadTransmitBuffer(uint8_t * Copy_puint8_tData, uint8_t Copy_uint8_tType) {

	/* We load the command buffer with the needed command MCP_COMMAND_LOAD_TX_BUFFER*/
	uint8_t Local_uint8_tCommandBuffer;
	if( Copy_uint8_tType == TRANSMIT_ID)
		Local_uint8_tCommandBuffer = MCP_COMMAND_LOAD_TX_BUFFER_ID;
	else if(Copy_uint8_tType == TRANSMIT_DATA)
		Local_uint8_tCommandBuffer = MCP_COMMAND_LOAD_TX_BUFFER_DATA;

	
	/* First we drive the CS pin low */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	
	/* We send the instruction packet MCP_COMMAND_LOAD_TX_BUFFER via SPI1 */
	HAL_SPI_Transmit(&hspi_1, &Local_uint8_tCommandBuffer, 1, HAL_MAX_DELAY);
		
	/* Then we send the data we wish to load the transmit buffer with */
	HAL_SPI_Transmit(&hspi_1, Copy_puint8_tData, 1, HAL_MAX_DELAY);
		
	/* CS pin is driven high once again to stop communication */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	
	
}

void delay_us(uint32_t delay_us)
{
  volatile unsigned int num;
  volatile unsigned int t;


  for (num = 0; num < delay_us; num++)
  {
    t = 11;
    while (t != 0)
    {
      t--;
    }
  }
}


void MCP2515_voidInit(void) {
	
	
	MCP_uint8_tMask = MODE_OF_OPERATION_MASK;
	MCP_uint8_tData = 0;
	
	
	/* First we reset the MCP */
	MCP2515_voidReset();
	delay_us(10);
	
	
	/* Sets bitrate to 500KBPs */
	MCP2515_voidSetBitRate();
	
	
	/* Enabling SOF ( Start Of Frame) Signal */
	uint8_t mask = ONE_BIT_MASK << CNF3_SOF;
	uint8_t data = 0x01 << CNF3_SOF;
	//MCP2515_voidBitModify(CNF3, &data, &mask);

	
	/* Disabling masks and filters */ 
	mask = TWO_BIT_MASK << RXB0CTRL_RXM0;
	data = 0x03 << RXB0CTRL_RXM0;
  MCP2515_voidBitModify(RXB0CTRL, &data, &mask);

	
#if   MODE_OF_OPERATION == NORMAL_MODE
				MCP_uint8_tData = NORMAL_MODE << CANCTRL_REQOP0;
			  MCP2515_voidBitModify(CANCTRL, &MCP_uint8_tData, &MCP_uint8_tMask); 
		
#elif MODE_OF_OPERATION == SLEEP_MODE
			  MCP_uint8_tData = SLEEP_MODE << CANCTRL_REQOP0;
			  MCP2515_voidBitModify(CANCTRL, &MCP_uint8_tData, &MCP_uint8_tMask); 
	
#elif MODE_OF_OPERATION == LOOPBACK_MODE
				MCP_uint8_tData = LOOPBACK_MODE << CANCTRL_REQOP0;
			  MCP2515_voidBitModify(CANCTRL, &MCP_uint8_tData, &MCP_uint8_tMask); 
			
#elif MODE_OF_OPERATION ==	LISTEN_ONLY_MODE
				MCP_uint8_tData = LISTEN_ONLY_MODE << CANCTRL_REQOP0;
			  MCP2515_voidBitModify(CANCTRL, &MCP_uint8_tData, &MCP_uint8_tMask); 

#elif MODE_OF_OPERATION == CONFIGURATION_MODE	
				MCP_uint8_tData = CONFIGURATION_MODE << CANCTRL_REQOP0;
			  MCP2515_voidBitModify(CANCTRL, &MCP_uint8_tData, &MCP_uint8_tMask); 
	
#endif	
		
}

void MCP2515_voidSendDataByte(uint8_t Copy_uint8_tDataByte, uint8_t Copy_uint8_tID) {
	
	uint8_t Local_uint8_tID = BOARD_ID;
	uint8_t Local_uint8_tRegisterBuffer = 0x00;
	uint8_t Local_uint8_tMaskBuffer = ONE_BIT_MASK << CANINTF_TX0IF;
	
	/* First we load the transmit buffer with the id */
	MCP2515_voidLoadTransmitBuffer(&Local_uint8_tID, TRANSMIT_ID);
	
	/* Then we load the transmit buffer with the data */
	MCP2515_voidLoadTransmitBuffer(&Copy_uint8_tDataByte, TRANSMIT_DATA);
	
	/* Finally we send the initiate transmission command */
	MCP2515_voidInitiateTransmission();
	
	uint8_t Local_uint8_Status = 0x00;
	
	while(Local_uint8_Status == 0) {
			MCP2515_uint8_tReadStatusByte(&Local_uint8_Status);
			Local_uint8_Status = Local_uint8_Status & 0x08;
		}
	
	

	/* We need to manually lower the transmit success flag TX0IF*/
	MCP2515_voidBitModify(CANINTF, &Local_uint8_tRegisterBuffer, &Local_uint8_tMaskBuffer);

		
}

uint8_t MCP2515_voidReceiveDataByte(uint8_t * Copy_puint8_tReceivedData,
									uint8_t Copy_uint8_tID,
									uint64_t Copy_uint64timeout) {
	
		uint8_t Local_uint8_tReceivedID = 0x00;
		uint8_t Local_uint8_tRegisterBuffer = 0x00;
		uint8_t Local_uint8_tMaskBuffer = ONE_BIT_MASK << CANINTF_RX0IF;
		uint32_t timeout = 0;
		uint8_t Local_uint8_Status = 0x00;
		while(Local_uint8_Status == 0 && timeout != Copy_uint64timeout) {
			MCP2515_uint8_tReadStatusByte(&Local_uint8_Status);
			Local_uint8_Status = Local_uint8_Status & 0x01;
			timeout++;
		}

		if(timeout == Copy_uint64timeout) {
			return NOOK;
		}

		else{
		/* First we read the ID of the received message */
		MCP2515_uint8_tReadReceiveBuffer(&Local_uint8_tReceivedID, TRANSMIT_ID);
		//HAL_UART_Transmit(&huart1, &Local_uint8_tReceivedID, 1, HAL_MAX_DELAY);
		
		//MCP2515_uint8_tReadReceiveBuffer(Copy_puint8_tReceivedData, TRANSMIT_DATA);
		//HAL_UART_Transmit(&huart1, Copy_puint8_tReceivedData, 1, HAL_MAX_DELAY);

		if( Local_uint8_tReceivedID == Copy_uint8_tID ) {
				MCP2515_uint8_tReadReceiveBuffer(Copy_puint8_tReceivedData, TRANSMIT_DATA);
				//HAL_UART_Transmit(&huart1, Copy_puint8_tReceivedData, 1, HAL_MAX_DELAY);
		}else {
				Copy_puint8_tReceivedData[0] = MCP_ERROR_MISCOMMUNICATION;
		}

	
		/* Then we compare it with out board's ID*/
		//if(Local_uint8_tReceivedID == BOARD_ID) {
			//MCP2515_uint8_tReadReceiveBuffer(Copy_puint8_tReceivedData, TRANSMIT_DATA);
		//} else {
			//*Copy_puint8_tReceivedData = 0x00;
		//}
	
			/* We need to manually lower the receive byte success flag RX0IF*/
		MCP2515_voidBitModify(CANINTF, &Local_uint8_tRegisterBuffer, &Local_uint8_tMaskBuffer);
		return OK;
	}
}


void MCP2515_voidSetBitRate(void) {
	
	/* This is a static configuration for a 125KBPs bit rate with an 8MHz clock*/

		uint8_t Local_uint8_tRegisterValue = 0x03;
		MCP2515_voidWrite(CNF1, &Local_uint8_tRegisterValue);
	
		Local_uint8_tRegisterValue = 0xF0;
		MCP2515_voidWrite(CNF2, &Local_uint8_tRegisterValue);

		Local_uint8_tRegisterValue = 0x86;
		MCP2515_voidWrite(CNF3, &Local_uint8_tRegisterValue);

	
	
//	uint8_t Local_uint8_tDataBuffer;
//	uint8_t Local_uint8_tMaskBuffer;
	
	/* CNF1 */
	
	/* Changing BRP to 4 */
	
	//Local_uint8_tMaskBuffer = SIX_BIT_MASK << CNF1_BPR0;
	//Local_uint8_tDataBuffer = 0x03  << CNF1_BPR0;
	
	//MCP2515_voidBitModify(CNF1, &Local_uint8_tDataBuffer, &Local_uint8_tMaskBuffer); 
	
	/* CNF2 */
	
	/* BTLMODE set to 1 */
	//Local_uint8_tDataBuffer = BTLMODE_VALUE << CNF2_BTLMODE; 	
	//Local_uint8_tMaskBuffer = BTLMODE_MASK;
	
	//MCP2515_voidBitModify(CNF2, &Local_uint8_tDataBuffer, &Local_uint8_tMaskBuffer); 
	
	
	/* Set PropSeg length to 2 TQs */
	//Local_uint8_tDataBuffer = TQ_PROPSEG << CNF2_PRSEG0;
	//Local_uint8_tMaskBuffer = PROPSEG_MASK;
	
	//MCP2515_voidBitModify(CNF2, &Local_uint8_tDataBuffer, &Local_uint8_tMaskBuffer); 


	/* Set PS1 length to 3 TQs */
	//Local_uint8_tDataBuffer = TQ_PS1 << CNF2_PHSEG1_0;
	//Local_uint8_tMaskBuffer = PS1_MASK;
	
	//MCP2515_voidBitModify(CNF2, &Local_uint8_tDataBuffer, &Local_uint8_tMaskBuffer); 
	
	
	/* Set PS2 to 2 TQs */
	//Local_uint8_tDataBuffer = TQ_PS2 << CNF3_PHSEG2_0;
	//Local_uint8_tMaskBuffer = PS2_MASK;
	
	//MCP2515_voidBitModify(CNF3, &Local_uint8_tDataBuffer, &Local_uint8_tMaskBuffer);

	
}










/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

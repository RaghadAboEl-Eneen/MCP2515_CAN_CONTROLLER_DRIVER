# MCP2515 CAN Controller Driver
 

## Table of contents
**[Introduction](#introduction)**<br>
**[How does the MCP2515 work?](#how-does-the-mcp2515-work)**<br>
**[Hardware and Pin Configurations](#hardware-and-pin-configurations)**<br>
**[Functions Overview](#functions-overview)**<br>
**[Functions Description](#functions-description)**<br>
**[References](#references)**<br>
**[Furthur improvments](#furthur-improvments)**<br>


---

## Introduction

The MCP2515 is a widely used SPI to CAN controller that allows microcontrollers that have no built-in CAN 
peripheral to use it as a communication protocol. Even a simple arduino UNO can be added to a CAN bus network which greatly
increases the range of microcontroller options for our projects. This driver is implemented using STM32CUBEMX and STM32CUBEIDE in the
C programming language.
	
[Jump back to the top](#table-of-contents)<br>

---


## How does the MCP2515 work?

MCP2515 is a CAN Controller that receives commands from us using SPI in a certain procedure and transmits and receives CAN frames. <br>
More details below.<br>
<br>
	
[Jump back to the top](#table-of-contents)<br>


---

## Hardware and Pin Configurations
* PA4:  SPI 1 CS   (Chip Select) 
* PA5:  SPI 1 SCK  (Source Clock)
* PA6:  SPI 1 MISO (Master In Slave Out)
* PA7:  SPI 1 MOSI (Master Out Slave In)
* PA9:  USART1 TX
* PA10: USART1 RX
* SPI Mode: Full-Duplex Master
* Hardware NSS Signal: Disabled
* Data Size: 8 Bits
* First Bit: MSB first (Most Significant Bit)
* Prescalar: 16
* Clock Polarity (CPOL): High
* Clock Phase (CPHA) : 2 Edge

<br>
	
[Jump back to the top](#table-of-contents)<br>


---

## Functions Overview
| Function                  | Descrition                                                                                          | 
| --------------------------|:---------------------------------------------------------------------------------------------------:| 
| `RESET`                   | Re-initalizes the internal registers of the MCP2515 and sets the operation mode to `CONFIGURATION`  | 
| `Initialize`              | Resets MCP2515 and adjusts registers that can only be accessed in `CONFIGURATION` mode              | 
| `READ`                    | Reads the value of a specified register in the MCP2515                                              | 
| `WRITE`                   | Writes to a specified register in the MCP2515                                                       |
| `BIT MODIFY`              | Writes to certain bits instead of all 8 bits in a specified register of the MCP2515                 | 
| `READ STATUS BYTE`        | Combines 8 flags that signal status of transmit and receive buffers into 1 Byte                     | 
| `READ RECEIVE BUFFER`     | Returns the value of the Receive Buffer specified (either buffer 0, 1 or 2)                         | 
| `LOAD TRANSMIT BUFFER`    | Loads the Transmit Buffer specified (either buffer 0 or 1) with the desired value                   |
| `INITIATE TRANSMISSION`   | Sends a command to the MCP2515 to begin CAN message transmission                                    | 
| `Send Data Byte CAN`      | Executes the needed procedure to fully send a data byte (more details below)                        |
| `Receive Data Byte CAN`   | Executes the needed procedure to fully read a data byte (more details below)                        |

<br>
	
[Jump back to the top](#table-of-contents)<br>
	
---

## Functions Description


### *RESET*

The `RESET` function can be used to reinitialize the internal registers of the MCP2515 and set the `CONFIGURATION` mode.<br><br> 
This command provides the same functionality, via the SPI interface, as the RESET pin.<br><br>
The RESET instruction is a single byte instruction that requires selecting the device by pulling the CS pin low,<br>
sending the instruction byte and then raising the CS pin.<br><br> 
It is highly recommended that the RESET command be sent (or the RESET pin be lowered) as part of the power-on initialization sequence.<br><br>

```c
void MCP2515_voidReset(void) {
		
	/* We load the command buffer with the needed command */
	uint8_t Local_uint8_tCommandBuffer = MCP_COMMAND_RESET;
	
	/* First we drive the CS pin low */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);

	/* Then we send the instruction packet MCP_COMMAND_RESET via SPI1 */
	HAL_SPI_Transmit(&hspi1, &Local_uint8_tCommandBuffer, 1, HAL_MAX_DELAY); 
	
	/* After packet is sent, CS pin is driven high once again */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	
}

```
### *Initialize*
#
### *READ*

The `READ` function is started by lowering the CS pin.<br><br>
The `READ` instruction is then sent to the MCP2515, followed by the 8-bit address (A7 through A0).<br><br>
Next, the data stored in the register at the selected address will be shifted out on the SO pin.<br><br>

```c
void MCP2515_voidRead(uint8_t Copy_uint8_tBaseAddress, uint8_t * Copy_puint8_tReceivedData) {
	
	/* We load the command buffer with the needed command MCP_COMMAND_READ */
	uint8_t Local_uint8_tCommandBuffer = MCP_COMMAND_READ;
		
	/* We load the base address buffer with the given base address */
	uint8_t Local_uint8_tBaseAddressBuffer = Copy_uint8_tBaseAddress;
		
	/* First we drive the CS pin low */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
		
	/* We send the instruction packet MCP_COMMAND_READ via SPI1 */
	HAL_SPI_Transmit(&hspi1, &Local_uint8_tCommandBuffer, 1, HAL_MAX_DELAY);
		
	/* We send the address of the register we want to read */
	HAL_SPI_Transmit(&hspi1, &Local_uint8_tBaseAddressBuffer, 1, HAL_MAX_DELAY);
	   
	/* We then receive the data sent over by the MCP via SPI1 */
	HAL_SPI_Receive(&hspi1, Copy_puint8_tReceivedData,1 , HAL_MAX_DELAY);
		
	/* CS pin is driven high once again to stop communication */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);		
	
}
```
#
### *WRITE*

The `WRITE` function is started by lowering the CS pin.<br><br>
The `WRITE` instruction is then sent to the MCP2515, followed by the address and at least one byte of data.<br><br>

```c
void MCP2515_voidWrite(uint8_t Copy_uint8_tBaseAddress, uint8_t * Copy_puint8_tData) {
	
	/* We load the command buffer with the needed command */
	uint8_t Local_uint8_tCommandBuffer = MCP_COMMAND_WRITE;
	
	/* We load the base address buffer with the given base address*/
	uint8_t Local_uint8_tBaseAddressBuffer = Copy_uint8_tBaseAddress;
	
	/* First we drive the CS pin low */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	
	/* We send the instruction packet MCP_COMMAND_WRITE via SPI1 */
	HAL_SPI_Transmit(&hspi1, &Local_uint8_tCommandBuffer, 1, HAL_MAX_DELAY);

	/* Then we send the address of the register we want to write to*/
	HAL_SPI_Transmit(&hspi1, &Local_uint8_tBaseAddressBuffer, 1, HAL_MAX_DELAY);
	
	/* Then we send the data we wish to write to the address */
	HAL_SPI_Transmit(&hspi1, Copy_puint8_tData, 1, HAL_MAX_DELAY);
	
  /* After packet is sent, CS pin is driven high once again */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);	

}
```
#
### *BIT MODIFY*
The `BIT MODIFY` function provides a means for setting or clearing individual bits in specific status and control registers.<br><br>

This command is not available for all registers.<br><br>

The instruction is selected by lowering the CS pin and the `BIT MODIFY` command byte is then sent to the MCP2515.<br><br>

The command is followed by the address of the register, the mask byte and finally, the data byte.<br><br>

The mask byte determines which bits in the register will be allowed to change.<br><br>

A ‘1’ in the mask byte will allow a bit in the register to change, while a ‘0’ will not.<br><br>

The data byte determines what value the modified bits in the register will be changed to.<br><br>

A ‘1’ in the data byte will set the bit and a ‘0’ will clear the bit, provided that the mask for that bit is set to a ‘1’.<br><br>

```c
void MCP2515_voidBitModify(uint8_t Copy_uint8_tBaseAddress, uint8_t * Copy_puint8_tData, uint8_t * Copy_puint8_tMask) {
	
	/* We load the command buffer with the needed command MCP_COMMAND_BIT_MODIFY*/
	uint8_t Local_uint8_tCommandBuffer = MCP_COMMAND_BIT_MODIFY;
	
	/* We load the base address buffer with the needed address*/
	uint8_t Local_uint8_tBaseAddressBuffer = Copy_uint8_tBaseAddress;

	/* First we drive the CS pin low */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	
	/* We send the instruction packet MCP_COMMAND_BIT_MODIFY via SPI1 */
	HAL_SPI_Transmit(&hspi1, &Local_uint8_tCommandBuffer, 1, HAL_MAX_DELAY);
	
	/* Then we send the address of the register we want to write to*/
	HAL_SPI_Transmit(&hspi1, &Local_uint8_tBaseAddressBuffer, 1, HAL_MAX_DELAY);
	
	/* Then we send the mask of the data */
	HAL_SPI_Transmit(&hspi1, Copy_puint8_tMask, 1, HAL_MAX_DELAY);
	
	/* Then we send the data itself */
	HAL_SPI_Transmit(&hspi1, Copy_puint8_tData, 1, HAL_MAX_DELAY);

	/* CS pin is driven high once again to stop communication */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	
}
```
#

### *READ STATUS BYTE*

The `READ STATUS BYTE` function allows single instruction access to some of the often used status bits for message reception and transmission.<br><br>

The MCP2515 is selected by lowering the CS pin and the `READ STATUS BYTE` command byte,is sent to the MCP2515.<br><br>

Once the command byte is sent, the MCP2515 will return eight bits of data that contain the status.<br><br>

Each status bit returned in this command may also be read by using the standard `READ` command with the appropriate register address.<br><br>

```c
void MCP2515_uint8_tReadStatusByte(uint8_t * Copy_puint8_tReceivedData) {
	
	/* We load the command buffer with the needed command MCP_COMMAND_READ_STATUS */
	uint8_t Local_uint8_tCommandBuffer[] = {MCP_COMMAND_READ_STATUS};
	
	/* First we drive the CS pin low */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	
	/* We send the instruction packet MCP_COMMAND_READ_STATUS via SPI1 */
	HAL_SPI_Transmit(&hspi1, Local_uint8_tCommandBuffer, 1, HAL_MAX_DELAY);
	
	/* We then receive the dats sent over by the MCP via SPI1 */
	HAL_SPI_Receive(&hspi1, Copy_puint8_tReceivedData, 1, HAL_MAX_DELAY);
	   
	/* CS pin is driven high once again to stop communication */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
  
}
```
#

### *READ RECEIVE BUFFER*

The `READ RX BUFFER` function provides a means to quickly address a receive buffer for reading.<br><br>

The command byte actually has four possible values that determine the Address Pointer location.<br>
Every Receive Buffer has a register for the "ID" and a register for the "DATA".<br><br>

Currently we are only using Receive Buffer 0.<br>
We specify in the `Copy_uint8_TYPE` whether it is "ID" or "DATA" we are trying to read.<br><br>

Once the command byte is sent, the controller clocks out the data at the address location, the same as the `READ` instruction.<br><br> 

```c
void MCP2515_uint8_tReadReceiveBuffer(uint8_t * Copy_puint8_tReceivedData, uint8_t Copy_uint8_tType) {
	
	uint8_t Local_uint8_tCommandBuffer;
	
	/* We load the command buffer with the needed command MCP_COMMAND_READ_RX_BUFFER*/
	if(Copy_uint8_tType == TRANSMIT_ID) 
		 Local_uint8_tCommandBuffer = MCP_COMMAND_READ_RX_BUFFER_ID;
	else if(Copy_uint8_tType == TRANSMIT_DATA) 
		 Local_uint8_tCommandBuffer = MCP_COMMAND_READ_RX_BUFFER_DATA;
	
	/* First we drive the CS pin low */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	
	/* We send the instruction packet MCP_COMMAND_READ_RX_BUFFER via SPI1 */
	HAL_SPI_Transmit(&hspi1, &Local_uint8_tCommandBuffer, 1, HAL_MAX_DELAY);
	
	/* We then receive the dats sent over by the MCP via SPI1 */
	HAL_SPI_Receive(&hspi1, Copy_puint8_tReceivedData, 1, HAL_MAX_DELAY);
	
	/* CS pin is driven high once again to stop communication */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);	
	
}
```
#

### *LOAD TRANSMIT BUFFER*

The `LOAD TX BUFFER` function eliminates the eight-bit address required by a normal `WRITE` command.<br><br>

The eight-bit instruction sets the Address Pointer to one of six addresses to quickly write to a transmit buffer<br> 

that points to the "ID" or "DATA" address of any of the three transmit buffers.<br><br>

Currently we are only using transmit buffer 0.<br>
We specify in the `Copy_uint8_TYPE` whether it is "ID" or "DATA" we are trying to load into the buffer.<br><br>

```c
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
	HAL_SPI_Transmit(&hspi1, &Local_uint8_tCommandBuffer, 1, HAL_MAX_DELAY);
		
	/* Then we send the data we wish to load the transmit buffer with */
	HAL_SPI_Transmit(&hspi1, Copy_puint8_tData, 1, HAL_MAX_DELAY);
		
	/* CS pin is driven high once again to stop communication */
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
		
}
```
#
### *INITIATE TRANSMISSION*


The RTS (request to send) instruction can be used to initiate message transmission for one or more transmit buffers.	

```c
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
```
#

### *Send Data Byte CAN*

The `Send Data Byte CAN` function makes use of the previous transmit functions. 

We specify the ID of the board we are sending a message to and load it into the transmit buffer.

Then the Databyte is loaded before transmission is intiaited.

After busy wait, the transmit success flag is lowered manually.


```c

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


```
#

### *Receive Data Byte CAN*


The `Receive Data Byte CAN` makes use of the previous receive functions.

The functions takes a pointer to store the received data, the ID of the board we want to receive data from and a timeout value to avoid a deadlock happening.

Since the ID is the first part of the message transmitted, it is read first and checked before reading the data itself.

After reading is complete, the receive success flag is lowered manually.

```c
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
		

		/* Then we compare it with out board's ID*/
		if( Local_uint8_tReceivedID == Copy_uint8_tID ) {
				MCP2515_uint8_tReadReceiveBuffer(Copy_puint8_tReceivedData, TRANSMIT_DATA);
		}else {
				Copy_puint8_tReceivedData[0] = MCP_ERROR_MISCOMMUNICATION;
		}

	
		/* We need to manually lower the receive byte success flag RX0IF*/
		MCP2515_voidBitModify(CANINTF, &Local_uint8_tRegisterBuffer, &Local_uint8_tMaskBuffer);
		return OK;
	}
}
```
#

---

## References
* [MCP2515 Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf)
* [CAN Specification Datasheet](http://esd.cs.ucr.edu/webres/can20.pdf)
* [Mastering STM32 by Carmine Noviello](https://www.carminenoviello.com/mastering-stm32/)
* [Arduino CAN tutorial - Interfacing MCP2515 CAN BUS Module with Arduino](https://circuitdigest.com/microcontroller-projects/arduino-can-tutorial-interfacing-mcp2515-can-bus-module-with-arduino)
<br>	
	
[Jump back to the top](#table-of-contents)<br>

	
---

## Furthur Improvments

* Enabling CRC for SPI

* Adding a function for remote frames (requesting data from another mcu)

* Checking and handling error flags of transmit and receive buffers

	
[Jump back to the top](#table-of-contents)<br>

---

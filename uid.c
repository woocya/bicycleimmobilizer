/*
 * uid.c
 *
 * Created: 14.02.2021 01:45:03
 *  Author: ja
 */ 
#define F_CPU 16000000UL
#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>
#include "add.h"
#include "uid.h"
#include "spi.h"
#include "LCD_HD44780_IIC.h"
#include "LCD_HD44780_IIC_config.h"



void PCD_WriteRegister(	uint8_t reg,	///< The register to write to. One of the PCD_Register enums.
uint8_t count,			///< The number of bytes to write to the register
uint8_t *values		///< The values to write. Byte array.
) {
	PORTB &= ~(1<<DD_SS);
//	SPI.beginTransaction(SPISettings(MFRC522_SPICLOCK, MSBFIRST, SPI_MODE0));	// Set the settings to work with SPI bus
//	digitalWrite(_chipSelectPin, LOW);		// Select slave
	SPI_MasterTransmit((reg<<1)&0x7E);						// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	for (uint8_t index = 0; index < count; index++) {
		SPI_MasterTransmit(values[index]);
	}
//	digitalWrite(_chipSelectPin, HIGH);		// Release slave again
//	SPI.endTransaction(); // Stop using the SPI bus
	PORTB |= (1<<DD_SS);
}

void PCD_ClearRegisterBitMask(	uint8_t reg,	///< The register to update. One of the PCD_Register enums.
uint8_t mask			///< The bits to clear.
) {
	uint8_t tmp;
	tmp = SPI_MasterReadOne(reg);
	SPI_MasterWrite(reg, tmp & (~mask));		// clear bit mask
}

uint8_t PCD_CalculateCRC(	uint8_t *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
uint8_t length,	///< In: The number of bytes to transfer.
uint8_t *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
) {
	SPI_MasterWrite(CommandReg, PCD_Idle);		// Stop any active command.
	SPI_MasterWrite(DivIrqReg, 0x04);	// Clear the CRCIRq interrupt request bit
	uint8_t n=SPI_MasterReadOne(FIFOLevelReg);
	SPI_MasterWrite(FIFOLevelReg,n|0x80);			
	//SPI_MasterWrite(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister(FIFODataReg, length, data);	// Write data to the FIFO
	SPI_MasterWrite(CommandReg, PCD_CalcCRC);		// Start the calculation
	
	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73?s.
	// TODO check/modify for other architectures than Arduino Uno 16bit

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73us.
	for (uint16_t i = 5000; i > 0; i--) {
		// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
		uint8_t n = SPI_MasterReadOne(DivIrqReg);
		if (n & 0x04) {									// CRCIRq bit set - calculation done
			SPI_MasterWrite(CommandReg, PCD_Idle);	// Stop calculating CRC for new content in the FIFO.
			// Transfer the result from the registers to the result buffer
			result[0] = SPI_MasterReadOne(CRCResultRegL);
			result[1] = SPI_MasterReadOne(CRCResultRegH);
			return STATUS_OK;
		}
	}
	// 89ms passed and nothing happend. Communication with the MFRC522 might be down.
	return STATUS_TIMEOUT;
}

uint8_t PCD_TransceiveData(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
uint8_t *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
) {
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}

uint8_t PCD_CommunicateWithPICC(	uint8_t command,		///< The command to execute. One of the PCD_Command enums.
uint8_t waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
uint8_t *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
) {
	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	
	SPI_MasterWrite(CommandReg, PCD_Idle);			// Stop any active command.
	SPI_MasterWrite(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	SPI_MasterWrite(FIFOLevelReg, 0x80);				// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	SPI_MasterWrite(BitFramingReg, bitFraming);		// Bit adjustments
	SPI_MasterWrite(CommandReg, command);				// Execute the command
	if (command == PCD_Transceive) {
		PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}
	
	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86?s.
	// TODO check/modify for other architectures than Arduino Uno 16bit
	uint16_t i;
	for (i = 2000; i > 0; i--) {
		uint8_t n = SPI_MasterReadOne(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
	}
	// 35.7ms and nothing happend. Communication with the MFRC522 might be down.
	if (i == 0) {
		return STATUS_TIMEOUT;
	}
	
	// Stop now if any errors except collisions were detected.
	uint8_t errorRegValue = SPI_MasterReadOne(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}
	
	uint8_t _validBits = 0;
	
	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		uint8_t n = SPI_MasterReadOne(FIFOLevelReg);	// Number of bytes in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;											// Number of bytes returned
		PCD_ReadRegister(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = SPI_MasterReadOne(ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}
	
	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		return STATUS_COLLISION;
	}
	
	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		uint8_t controlBuffer[2];
		uint8_t status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}
	
	return STATUS_OK;
}
void PCD_ReadRegister(	uint8_t reg,	///< The register to read from. One of the PCD_Register enums.
								uint8_t count,			///< The number of bytes to read
								uint8_t *values,		///< Byte array to store the values in.
								uint8_t rxAlign		///< Only bit positions rxAlign..7 in values[0] are updated.
								) {
	if (count == 0) {
		return;
	}
	//Serial.print(F("Reading ")); 	Serial.print(count); Serial.println(F(" bytes from register."));
	
	uint8_t address = ((reg<<1)&0x7E)|0x80;				// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	uint8_t index = 0;							// Index in values array.
//	SPI.beginTransaction(SPISettings(MFRC522_SPICLOCK, MSBFIRST, SPI_MODE0));	// Set the settings to work with SPI bus
//	digitalWrite(_chipSelectPin, LOW);		// Select slave
	PORTB &= ~(1<<DD_SS);
	count--;								// One read is performed outside of the loop
	SPI_MasterTransmit(address);					// Tell MFRC522 which address we want to read
	if (rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
		// Create bit mask for bit positions rxAlign..7
		uint8_t mask = (0xFF << rxAlign) & 0xFF;
		// Read value and tell that we want to read the same address again.
		uint8_t value = SPI_MasterTransmit(address);
		// Apply mask to both current value of values[0] and the new data in value.
		values[0] = (values[0] & ~mask) | (value & mask);
		index++;
	}
	while (index < count) {
		values[index] = SPI_MasterTransmit(address);	// Read value and tell that we want to read the same address again.
		index++;
	}
	values[index] = SPI_MasterTransmit(0);			// Read the final byte. Send 0 to stop reading.
	PORTB |= (1<<DD_SS);
//	digitalWrite(_chipSelectPin, HIGH);			// Release slave again
//	SPI.endTransaction(); // Stop using the SPI bus
} // End PCD_ReadRegister()

/**
 * Sets the bits given in mask in register reg.
 */

uint8_t PICC_Select(	Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
uint8_t validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
) {
	bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	uint8_t cascadeLevel = 1;
	uint8_t result;
	uint8_t count;
	uint8_t checkBit;
	uint8_t index;
	uint8_t uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	uint8_t bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	uint8_t rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte.
	uint8_t *responseBuffer;
	uint8_t responseLength;

	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits.
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9
	
	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}

	// Prepare MFRC522
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	
	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
			buffer[0] = PICC_CMD_SEL_CL1;
			uidIndex = 0;
			useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
			break;
			
			case 2:
			buffer[0] = PICC_CMD_SEL_CL2;
			uidIndex = 3;
			useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
			break;
			
			case 3:
			buffer[0] = PICC_CMD_SEL_CL3;
			uidIndex = 6;
			useCascadeTag = false;						// Never used in CL3.
			break;
			
			default:
			return STATUS_INTERNAL_ERROR;
			break;
		}
		
		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}
		
		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while (!selectDone) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				
				// Calculate CRC_A
				result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != STATUS_OK) {
					return result;
				}
				
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}
			
			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			SPI_MasterWrite(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			bool checkCRC=false;
			// Transmit the buffer and receive the response.
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, checkCRC);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				uint8_t valueOfCollReg = SPI_MasterReadOne(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits	= collisionPos;
				count			= currentLevelKnownBits % 8; // The bit to modify
				checkBit		= (currentLevelKnownBits - 1) % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << checkBit);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)
		
		// We do not check the CBB - it was constructed by us above.
		
		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}
		
		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)
	
	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;


	return STATUS_OK;
} // End PICC_Select()

uint8_t PICC_ReadCardSerial() {
	uint8_t result = PICC_Select(&uid, 0);
	return result;
}
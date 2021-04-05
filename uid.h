/*
 * uid.h
 *
 * Created: 14.02.2021 02:01:08
 *  Author: ja
 */ 


#ifndef UID_H_
#define UID_H_

typedef enum {false, true} bool; 

typedef struct {
	uint8_t		size;			// Number of bytes in the UID. 4, 7 or 10.
	uint8_t		uidByte[4];
	uint8_t		sak;			// The SAK (Select acknowledge) byte returned from the PICC after successful selection.
} Uid;

Uid uid;


void PCD_ClearRegisterBitMask(	uint8_t reg,	///< The register to update. One of the PCD_Register enums.
uint8_t mask			///< The bits to clear.
);

uint8_t PCD_CalculateCRC(	uint8_t *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
uint8_t length,	///< In: The number of bytes to transfer.
uint8_t *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
);

uint8_t PCD_TransceiveData(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
uint8_t *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
);

uint8_t PCD_CommunicateWithPICC(	uint8_t command,		///< The command to execute. One of the PCD_Command enums.
uint8_t waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
uint8_t *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
);

void PCD_ReadRegister(	uint8_t reg,	///< The register to read from. One of the PCD_Register enums.
uint8_t count,			///< The number of bytes to read
uint8_t *values,		///< Byte array to store the values in.
uint8_t rxAlign		///< Only bit positions rxAlign..7 in values[0] are updated.
);

void PCD_SetRegisterBitMask(	uint8_t reg,	///< The register to update. One of the PCD_Register enums.
uint8_t mask			///< The bits to set.
);

uint8_t PICC_Select(	Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
uint8_t validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
);

uint8_t PICC_ReadCardSerial();
#endif /* UID_H_ */
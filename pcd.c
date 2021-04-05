/*
 * pcd.c
 *
 * Created: 21.02.2021 20:39:29
 *  Author: ja
 */ 

#include "pcd.h"

void PCD_Reset() {
	SPI_MasterWrite(CommandReg, PCD_SoftReset);	// Issue the SoftReset command.
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg)
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74?s. Let us be generous: 50ms.
	uint8_t n=0;
	uint8_t count = 0;
	do {
		// Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
		_delay_ms(50);
		n=SPI_MasterReadOne(CommandReg);
	} while ((n & (1 << 4)) && (++count) < 3);
}


void PCD_Init() {
	
	PCD_Reset();

	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	SPI_MasterWrite(TModeReg, 0x8D); // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	SPI_MasterWrite(TPrescalerReg, 0x3E ); // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25?s.
	SPI_MasterWrite(TReloadRegH, 0x1E);//	TReloadRegH, 0x03	// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	SPI_MasterWrite(TReloadRegL, 0x00);// TReloadRegL, 0xE8
	
	SPI_MasterWrite(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	SPI_MasterWrite(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	uint8_t value=0;
	value=SPI_MasterReadOne(TxControlReg);
	if ((value & 0x03) != 0x03) {
		SPI_MasterWrite(TxControlReg, value | 0x03);
		SPI_MasterReadOne(TxControlReg);					// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
	}
}

void PCD_SetRegisterBitMask(uint8_t reg,	///< The register to update.
uint8_t mask)			///< The bits to set
{
	uint8_t tmp;
	tmp = SPI_MasterReadOne(reg);
	SPI_MasterWrite(reg, tmp | mask);			// set bit mask
}
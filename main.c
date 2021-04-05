/*
 * 16.02.c
 *
 * Created: 16.02.2021 12:25:52
 * Author : ja
 */ 
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "add.h"
#include "LCD_HD44780_IIC.h"
#include "uid.h"
#include "spi.h"
#include "pcd.h"




int main(void)
{
	
  	uint8_t waitIRq=0x30;
	uint8_t bufferATQA[16];
	uint8_t bufferSize = sizeof(bufferATQA);
	uint8_t n;
	uint8_t rxAlign=0;
	uint8_t validBits=7;
	uint8_t status=STATUS_ERROR;
	uint8_t accepted[4];
	int cardSaved=0;
	int on_off=0;
	int a;
	
	for (int b=0;b<4;b++)
		uid.uidByte[b]=0;
		
	
	LCDinit();
	_delay_ms(10);
	LCDhome();
	_delay_ms(10);
	
	LCDstring("Closed", 6);
	_delay_ms(10);
	
	SPI_MasterInit();
	PCD_Init();
	
	uint8_t bitFraming = (rxAlign << 4) + validBits;
	n = SPI_MasterReadOne(ComIEnReg);
	SPI_MasterWrite(ComIEnReg,n|0x20);
	n = SPI_MasterReadOne(DivIEnReg);
	SPI_MasterWrite(DivIEnReg,n|0x80);
	
	while(1)
	{
		uint8_t n;
		uint16_t i;
		
		SPI_MasterWrite(CommandReg, PCD_Idle);  // send to CommandReg (0x01) PCD_Idle (0x00) and return error 0x02 - preparation of the module, stops any execution
		n=SPI_MasterReadOne(ComIrqReg);
		SPI_MasterWrite(ComIrqReg,n&(~0x80)); //clear all interrupt bits
		n=SPI_MasterReadOne(FIFOLevelReg);
		SPI_MasterWrite(FIFOLevelReg,n|0x80); //flush FIFO data
		SPI_MasterWrite(FIFODataReg, PICC_CMD_WUPA);// Write sendData to the FIFO
		SPI_MasterWrite(BitFramingReg, bitFraming);  // Bit adjustments
		SPI_MasterWrite(CommandReg, PCD_Transceive);  // send data to the antenna
		PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts	
			
		for (i = 2000; i > 0; i--)
		{
			n=0x00;
			n=SPI_MasterReadOne(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
			if (n & waitIRq) {	// One of the interrupts that signal success has been set.
				status = STATUS_OK;
				break;
			}
			
			if (n & 0x01) {					// Timer interrupt - nothing received in 25ms
				status = STATUS_TIMEOUT;
			}
		}
		
		// 35.7ms and nothing happened. Communication with the MFRC522 might be down.
		if (i == 0) {
			status = STATUS_TIMEOUT;
		}
		
		n=SPI_MasterReadOne(BitFramingReg);
		SPI_MasterWrite(BitFramingReg,n&(~0x80));
		
		n = SPI_MasterReadOne(ErrorReg);	
		if (n & 0x13)  // BufferOvfl ParityErr ProtocolErr
		{
			status = STATUS_ERROR;
		}
			
		n=SPI_MasterReadOne(FIFOLevelReg);  // Number of bytes in the FIFO
		if (n > bufferSize) {
			status = STATUS_NO_ROOM;
		}
		
		for (int x=0; x<bufferSize; x++)
			bufferATQA[x] = SPI_MasterReadOne(FIFODataReg); // Get received data from FIFO
		
		uint8_t _validBits = SPI_MasterReadOne(ControlReg);		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		
		_validBits = _validBits & 0x07;
		if (_validBits!=0)
		{
			status = STATUS_ERROR;
		}
		
		// Tell about collisions
		n = SPI_MasterReadOne(ErrorReg);
		if (n & 0x08)	// CollErr
		{
			status = STATUS_COLLISION;
		}

		if (status == STATUS_OK)
		{

			PICC_ReadCardSerial();

			if (cardSaved==0)
			{
				for (a=0;a<4;a++)
					accepted[a]=uid.uidByte[a];
				LCDclr();
				LCDGotoXY(0,0);
				_delay_ms(10);
				LCDstring("card saved", 10);
				_delay_ms(1000);
				cardSaved++;
			}
			
			for (a=0;a<4;a++)
			{
				if (uid.uidByte[a] != accepted[a])
				{
					LCDclr();
					LCDGotoXY(0,0);
					_delay_ms(10);
					LCDstring("access denied", 13);
					_delay_ms(1000);
		
					status=STATUS_WRONG_CARD;
					break;
				}
			}
	
		}
			
		if (status==STATUS_OK)
		{
			if (on_off==0)
			{
				LCDclr();
				LCDGotoXY(0,0);
				LCDstring("Closed", 6);
				DDRD |= (1 << DDD3);
				PORTD |= (1 << PORTD3);
				_delay_ms(1000);
				on_off++;
			}
			else
			{
				LCDclr();
				LCDGotoXY(0,0);
				LCDstring("Opened", 6);
				PORTD &= ~(1 << PORTD3);
				_delay_ms(1000);
				on_off=0;
			}
		}
	}
}




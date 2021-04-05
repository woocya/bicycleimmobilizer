/*
 * spi.c
 *
 * Created: 16.02.2021 13:54:08
 *  Author: ja
 */ 
#include "spi.h"
#include "add.h"

void SPI_MasterInit(void)
{
	/* Set MOSI and SCK output, all others input */
	DDR_SPI = (1<<DD_MOSI)|(1<<DD_SCK)|(1<<DD_SS);
	/* Enable SPI, Master, set clock rate fck/16 */
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}

uint8_t SPI_MasterTransmit(uint8_t data)
{
	SPDR = data;
	while(!(SPSR & (1<<SPIF))); // spif bit signals successful transmission
	
	return SPDR;
}


void SPI_MasterWrite(uint8_t reg, uint8_t data)
{
	PORTB &= ~(1<<DD_SS);
	SPI_MasterTransmit((reg<<1)&0x7E); // reg shifted 1 position to left because of data frame used by rc522, 0 on msb to write
	SPI_MasterTransmit(data);
	PORTB |= (1<<DD_SS);
	
	
}

uint8_t SPI_MasterReadOne(uint8_t reg)
{
	uint8_t data;
	PORTB &= ~(1<<DD_SS);
	SPI_MasterTransmit(((reg<<1)&0x7E)|0x80); // reg shifted 1 position to left because of data frame used by rc522, 1 on msb to read
	data = SPI_MasterTransmit(0x00);
	PORTB |= (1<<DD_SS);
	return data;

}
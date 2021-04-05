/*
 * spi.h
 *
 * Created: 16.02.2021 13:54:53
 *  Author: ja
 */ 


#ifndef SPI_H_
#define SPI_H_

#include <avr/io.h>
#include <string.h>
#include <util/delay.h>

#define DD_SCK DDB7
#define DD_MOSI DDB5
#define DD_SS DDB4
#define DDR_SPI DDRB

void SPI_MasterInit();
uint8_t SPI_MasterTransmit(uint8_t data);
void SPI_MasterWrite(uint8_t reg, uint8_t data);
uint8_t SPI_MasterReadOne(uint8_t reg);



#endif /* SPI_H_ */
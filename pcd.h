/*
 * pcd.h
 *
 * Created: 21.02.2021 20:39:56
 *  Author: ja
 */ 


#ifndef PCD_H_
#define PCD_H_

#define F_CPU 16000000UL
#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>
#include "add.h"
#include "uid.h"
#include "spi.h"

void PCD_Reset();
void PCD_Init();
void PCD_SetRegisterBitMask(uint8_t reg, uint8_t mask);



#endif /* PCD_H_ */
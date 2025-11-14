/*
 * max30102_driver.h
 *
 *  Created on: Oct 30, 2025
 *      Author: himethaluthwala
 */

#ifndef INC_MAX30102_DRIVER_H_
#define INC_MAX30102_DRIVER_H_


#include "stm32f407xx.h"


/********************************************************************************/

/* Register definition structure */

/********************************************************************************/

typedef struct
{
	volatile uint8_t IS1;						/* MAX30102 interrupt status register 1			Offset: 0x00 */
	volatile uint8_t IS2;						/* MAX30102 interrupt status register 2			Offset: 0x01 */
	volatile uint8_t IE1;						/* MAX30102 interrupt enable register 1			Offset: 0x02 */
	volatile uint8_t IE2;						/* MAX30102 interrupt enable register 2			Offset: 0x03 */
	volatile uint8_t FIFOWP;					/* MAX30102 FIFO write pointer 					Offset: 0x04 */
	volatile uint8_t OVFC;						/* MAX30102 overflow counter 					Offset: 0x05 */
	volatile uint8_t FIFORP;					/* MAX30102 FIFO read pointer 					Offset: 0x06 */
	volatile uint8_t FIFODR;					/* MAX30102 FIFO data register					Offset: 0x07 */
	volatile uint8_t FIFOCFGR;					/* MAX30102 FIFO configuration register			Offset: 0x08 */
	volatile uint8_t MODECFGR;					/* MAX30102 mode configuration register			Offset: 0x09 */
	volatile uint8_t SPO2CFGR;					/* MAX30102 SpO2 configuration register			Offset: 0x0A */
	volatile uint8_t RESERVED0;					/* 												Offset: 0x0B */
	volatile uint8_t LED1PA;					/* MAX30102 LED1 pulse amplitude register		Offset: 0x0C */
	volatile uint8_t LED2PA;					/* MAX30102 LED2 pulse amplitude register		Offset: 0x0D */
	volatile uint8_t RESERVED1;					/* 												Offset: 0x0E */
	volatile uint8_t RESERVED2;					/* 												Offset: 0x0F */
	volatile uint8_t RESERVED3;					/* 												Offset: 0x10 */
	volatile uint8_t MLEDCR1;					/* MAX30102 multi-LED control register 1		Offset: 0x11 */
	volatile uint8_t MLEDCR2;					/* MAX30102 multi-LED control register 2		Offset: 0x12 */
	volatile uint8_t RESERVED4;					/* 												Offset: 0x13 */
	volatile uint8_t RESERVED5;					/* 												Offset: 0x14 */
	volatile uint8_t RESERVED6;					/* 												Offset: 0x15 */
	volatile uint8_t RESERVED7;					/* 												Offset: 0x16 */
	volatile uint8_t RESERVED9;					/* 												Offset: 0x17 */
	volatile uint8_t RESERVED10;				/* 												Offset: 0x18 */
	volatile uint8_t RESERVED11;				/* 												Offset: 0x19 */
	volatile uint8_t RESERVED12;				/* 												Offset: 0x1A */
	volatile uint8_t RESERVED13;				/* 												Offset: 0x1B */
	volatile uint8_t RESERVED14;				/* 												Offset: 0x1C */
	volatile uint8_t RESERVED15;				/* 												Offset: 0x1D */
	volatile uint8_t RESERVED16;				/* 												Offset: 0x1E */
	volatile uint8_t DTI;						/* MAX30102 die temperature integer				Offset: 0x1F */
	volatile uint8_t DTF;						/* MAX30102 die temperature fraction			Offset: 0x20 */
	volatile uint8_t DTC;						/* MAX30102 die temperature configuration		Offset: 0x21 */
}MAX30102_RegDef_t;


/********************************************************************************/

/* Configuration structure */

/********************************************************************************/

typedef struct
{
	uint8_t Sensor_Mode;
	uint8_t SpO2_SampleRate;
	uint8_t LED_PulseWidth;
	uint8_t LED_Current;
}MAX30102_Config_t;


/********************************************************************************/

/* Handle structure */

/********************************************************************************/

typedef struct
{
	MAX30102_RegDef_t* pMAX30102;
	MAX30102_Config_t  MAX30102_Config;
};





#endif /* INC_MAX30102_DRIVER_H_ */

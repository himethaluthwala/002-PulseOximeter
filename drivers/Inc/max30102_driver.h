/*
 * max30102_driver.h
 *
 *  Created on: Oct 30, 2025
 *      Author: himethaluthwala
 */

#ifndef INC_MAX30102_DRIVER_H_
#define INC_MAX30102_DRIVER_H_


#include "stm32f407xx.h"


/*
 * MAX30102 sensor slave address
 */

#define MAX30102_SLAVEADDR		0x57
#define MAX30102_PARTID			0x15


/********************************************************************************/

/* MAX30102 register addresses */

/********************************************************************************/

#define FIFO_WP			0x04
#define FIFO_OVFC		0x05
#define FIFO_RP			0x06
#define FIFO_DR			0x07
#define FIFO_CFGR		0x08
#define MODE_CFGR		0x09
#define SPO2_CFGR		0x0A
#define LED1_PA			0x0C
#define LED2_PA			0x0D
#define PART_ID			0xFF


/********************************************************************************/

/* Configuration structure */

/********************************************************************************/

typedef struct
{
	uint8_t Sensor_Mode;			// Values from @SENSOR_MODE
	uint8_t SpO2_SampleRate;		// Values from @SAMPLE_RATE
	uint8_t LED_PulseWidth;			// Values from @PULSE_WIDTH
	uint8_t SpO2_ADCRange;			// Values from @ADC_RANGE
}MAX30102_Config_t;


/********************************************************************************/

/* Sample buffer structure */

/********************************************************************************/

typedef struct
{
    uint32_t red[32];
    uint32_t ir[32];
    uint8_t count;
}MAX30102_SampleBuffer_t;


/********************************************************************************/

/* Handle structure */

/********************************************************************************/

typedef struct
{
	I2C_Handle_t				I2CHandle;
	MAX30102_Config_t  			MAX30102_Config;
	MAX30102_SampleBuffer_t		SampleBuffer;
}MAX30102_Handle_t;


/* @SENSOR_MODE */

#define MAX30102_MODE_HR			2
#define MAX30102_MODE_SPO2			3


/* @SAMPLE_RATE */

#define MAX30102_SR_50HZ			0
#define MAX30102_SR_100HZ			1
#define MAX30102_SR_200HZ			2
#define MAX30102_SR_400HZ			3
#define MAX30102_SR_800HZ			4
#define MAX30102_SR_1000HZ			5
#define MAX30102_SR_1600HZ			6
#define MAX30102_SR_3200HZ			7


/* @PULSE_WIDTH */

#define MAX30102_PW_69US			0
#define MAX30102_PW_118US			1
#define MAX30102_PW_215US			2
#define MAX30102_PW_411US			3


/* @ADC_RANGE */

#define MAX30102_ADCR_2048NA		0
#define MAX30102_ADCR_4096NA 		1
#define MAX30102_ADCR_8192NA 		2
#define MAX30102_ADCR_16384NA 		3


/* @LED_CURRENT */

#define MAX30102_LEDCURR_12MA		0x1C


/********************************************************************************/

/* Bit position definitions of MAX30102 registers */

/********************************************************************************/

/* Mode configuration */

#define MAX30102_MODECFGR_MODE			0
#define MAX30102_MODECFGR_RESET			6
#define MAX30102_MODECFGR_SHDN			7


/* SpO2 configuration */

#define MAX30102_SPO2CFGR_LED_PW		0
#define MAX30102_SPO2CFGR_SR			2
#define MAX30102_SPO2CFGR_ADC_RGE		5

/* FIFO configuration */

#define MAX30102_FIFOCFGR_FIFO_AFULL	0
#define MAX30102_FIFOCFGR_FIFO_ROE		4
#define MAX30102_FIFOCFGR_SMP_AVG		5


/********************************************************************************/

/* APIs supported by this driver */

/********************************************************************************/

void MAX30102_Init(MAX30102_Handle_t* pMAXHandle);
void MAX30102_Reset(MAX30102_Handle_t* pMAXHandle);

uint8_t MAX30102_FIFORead(MAX30102_Handle_t* pMAXHandle);




#endif /* INC_MAX30102_DRIVER_H_ */

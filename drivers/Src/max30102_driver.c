/*
 * max30102_driver.c
 *
 *  Created on: Oct 30, 2025
 *      Author: himethaluthwala
 */


#include "max30102_driver.h"

static void WriteRegister(MAX30102_Handle_t* pMAXHandle, uint8_t reg, uint8_t value, uint8_t Sr);
static uint8_t ReadRegister(MAX30102_Handle_t* pMAXHandle, uint8_t reg, uint8_t Sr);
static void ReadSample(MAX30102_Handle_t* pMAXHandle, uint32_t index);


static void WriteRegister(MAX30102_Handle_t* pMAXHandle, uint8_t reg, uint8_t value, uint8_t Sr) {

	uint8_t data[2] = {reg, value};

	I2C_MasterSendData(&pMAXHandle->I2CHandle, data, sizeof(data), MAX30102_SLAVEADDR, Sr);

}


static uint8_t ReadRegister(MAX30102_Handle_t* pMAXHandle, uint8_t reg, uint8_t Sr) {

	uint8_t value;

	I2C_MasterSendData(&pMAXHandle->I2CHandle, &reg, 1, MAX30102_SLAVEADDR, ENABLE);
	I2C_MasterReceiveData(&pMAXHandle->I2CHandle, &value, 1, MAX30102_SLAVEADDR, Sr);

	return value;

}

static void ReadSample(MAX30102_Handle_t* pMAXHandle, uint32_t index) {

	uint32_t temp = 0;

	// build red sample
	temp |= ((uint32_t)ReadRegister(pMAXHandle, FIFO_DR, ENABLE) << 16);
	temp |= ((uint32_t)ReadRegister(pMAXHandle, FIFO_DR, ENABLE) << 8);
	temp |= (uint32_t)ReadRegister(pMAXHandle, FIFO_DR, ENABLE);
	temp &= 0x3FFFF; 		// clear last 6 bits

	pMAXHandle->SampleBuffer.red[index] = temp;

	// build IR sample
	temp = 0;
	temp |= ((uint32_t)ReadRegister(pMAXHandle, FIFO_DR, ENABLE) << 16);
	temp |= ((uint32_t)ReadRegister(pMAXHandle, FIFO_DR, ENABLE) << 8);
	temp |= (uint32_t)ReadRegister(pMAXHandle, FIFO_DR, ENABLE);
	temp &= 0x3FFFF; 		// clear last 6 bits

	pMAXHandle->SampleBuffer.ir[index] = temp;

}


void MAX30102_Init(MAX30102_Handle_t* pMAXHandle) {

	// confirm part ID
	if (MAX30102_PARTID != ReadRegister(pMAXHandle, PART_ID, ENABLE)) {
		// error...
	}

	uint8_t tempreg = 0;

	// reset sensor
	WriteRegister(pMAXHandle, MODE_CFGR, (1 << MAX30102_MODECFGR_RESET), ENABLE);

	// wait until RESET bit clears
	while (ReadRegister(pMAXHandle, MODE_CFGR, ENABLE) & (1 << MAX30102_MODECFGR_RESET));

	// reset FIFO pointers
	WriteRegister(pMAXHandle, FIFO_WP, 0x00, ENABLE);
	WriteRegister(pMAXHandle, FIFO_RP, 0x00, ENABLE);
	WriteRegister(pMAXHandle, FIFO_OVFC, 0x00, ENABLE);

	// configure FIFO
	tempreg |= (0x0F << MAX30102_FIFOCFGR_FIFO_AFULL);		// FIFO almost full = 15
	tempreg |= (2 << MAX30102_FIFOCFGR_SMP_AVG); 	  		// 4 samples averaged per FIFO sample
	tempreg |= (1 << MAX30102_FIFOCFGR_FIFO_ROE); 	  		// FIFO roll over enabled
	WriteRegister(pMAXHandle, FIFO_CFGR, tempreg, ENABLE);

	// configure SPO2 settings
	tempreg = 0;
	tempreg |= (pMAXHandle->MAX30102_Config.SpO2_SampleRate << MAX30102_SPO2CFGR_SR);
	tempreg |= (pMAXHandle->MAX30102_Config.LED_PulseWidth << MAX30102_SPO2CFGR_LED_PW);
	tempreg |= (MAX30102_ADCR_8192NA << MAX30102_SPO2CFGR_ADC_RGE);
	WriteRegister(pMAXHandle, SPO2_CFGR, tempreg, ENABLE);

	// configure LED current
	WriteRegister(pMAXHandle, LED1_PA, MAX30102_LEDCURR_12MA, ENABLE);
	WriteRegister(pMAXHandle, LED2_PA, MAX30102_LEDCURR_12MA, ENABLE);

	// configure sensor mode
	tempreg = 0;
	tempreg = ReadRegister(pMAXHandle, MODE_CFGR, ENABLE);
	tempreg &= 0xF8; 		// clears last 3 bits
	tempreg |= (pMAXHandle->MAX30102_Config.Sensor_Mode << MAX30102_MODECFGR_MODE);
	tempreg &= ~(1 << MAX30102_MODECFGR_RESET); 	// clear RESET bit
	WriteRegister(pMAXHandle, MODE_CFGR, tempreg, DISABLE);

}


uint8_t MAX30102_FIFORead(MAX30102_Handle_t* pMAXHandle) {

	uint8_t write_ptr = ReadRegister(pMAXHandle, FIFO_WP, ENABLE);
	uint8_t read_ptr = ReadRegister(pMAXHandle, FIFO_RP, DISABLE);

	uint32_t num_samples = write_ptr - read_ptr;

	if (num_samples == 0) {
		return 0;
	}

	if (num_samples < 0) {
		num_samples += 32; 		// FIFO is circular
	}

	for (uint32_t i = 0; i < num_samples; i++) {
		ReadSample(pMAXHandle, i);
	}

	return pMAXHandle->SampleBuffer.count;

}


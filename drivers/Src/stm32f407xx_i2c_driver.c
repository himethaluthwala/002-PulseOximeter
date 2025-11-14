/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Sep 20, 2025
 *      Author: himethaluthwala
 */


#include "stm32f407xx_i2c_driver.h"


uint16_t AHB1Prescalars[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t  APB1Prescalars[4] = {2, 4, 8, 16};

// helper functions

static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr, uint8_t RorW);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXNEInterrupt(I2C_Handle_t *pI2CHandle);


static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx) {

	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx) {

	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr, uint8_t RorW) {

	SlaveAddr = (SlaveAddr << 1);		// shift left to make space for r/w bit
	if (RorW == 1) { 		// read
		SlaveAddr |= (1);
	} else {				// write
		SlaveAddr &= ~(1);
	}
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle) {

	uint32_t dummy_read;
	// check for device mode
	if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
		// device is in master mode
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			if (pI2CHandle->RxSize == 1) {
				// first disable the ACK
				I2C_ManageACK(pI2CHandle->pI2Cx, DISABLE);

				// clear ADDR flag (read SR1, read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		} else {
			// can clear ADDR flag without disabling ACK
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
		}
	} else {
		// device is in slave mode
		// can just clear the ADDR flag
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
	}
}

uint32_t RCC_GetPLLOutputClock() {

	return 0;
}


uint32_t RCC_GetPCL1Value(void) {

	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahb1p, apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);		// right shifting values in CGFR by 2 and masking all bits except the first 2

	if (clksrc == 0) {
		SystemClk = 16000000;
	}
	else if (clksrc == 1) {
		SystemClk = 8000000;
	}
	else if (clksrc == 2) {
		SystemClk = RCC_GetPLLOutputClock();
	}

	// for AHB1
	temp = ((RCC->CFGR >> 4) & 0xF);		// right shifting values in CGFR by 4 and masking all bits except the first 4

	if (temp < 8) {
		ahb1p = 1;
	} else {
		ahb1p = AHB1Prescalars[temp-8];
	}

	// for APB1
	temp = ((RCC->CFGR >> 10) & 0x7);		// right shifting values in CGFR by 10 and masking all bits except the first 3

	if (temp < 4) {
		apb1p = 1;
	} else {
		apb1p = APB1Prescalars[temp-4];
	}

	pclk1 = (SystemClk / ahb1p) / apb1p;

	return pclk1;
}


/*
 * Function name:			I2C_Init
 *
 * Description:				Initialises the I2C peripheral
 *
 * Parameter 1:				Handler that contains base address of the I2C peripheral and configuration settings
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void I2C_Init(I2C_Handle_t *pI2CHandle) {

	uint32_t tempreg = 0;
	uint8_t trise;

	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// enable the ACK control bit
	tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// configure the FREQ field in CR2
	tempreg = 0;
	tempreg |= (RCC_GetPCL1Value() / 1000000U);		// to reduce to just 16 instead of 16 MHz
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);		// to mask all other bits except the first 5

	// program the device's own address
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);		// to not occupy the 1st bit
	tempreg |= (1 << 14);											// 14th bit must be kept at 1 (mentioned in reference manual)
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// configure the speed of the serial clock
	uint16_t ccr_value = 0;
	tempreg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM) {
		// standard mode
		ccr_value = (RCC_GetPCL1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);		// mask out everything except the first 12 bits
	}
	else {
		// fast mode
		tempreg |= (1 << I2C_CCR_FS);		// turn on fast mode bit
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);	// configure duty cycle

		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccr_value = (RCC_GetPCL1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		} else {
			ccr_value = (RCC_GetPCL1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	// configure the rise time for the I2C pin
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM) {
		// standard mode
		trise = (RCC_GetPCL1Value() / 1000000U) + 1;
	}
	else {
		// fast mode
		trise = ((RCC_GetPCL1Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (trise & 0x3F);		// masked all other bits except first 5
}


/*
 * Function name:			I2C_DeInit
 *
 * Description:				Deinitialises the I2C peripheral
 *
 * Parameter 1:				Base address of the I2C peripheral
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void I2C_DeInit(I2C_RegDef_t *pI2Cx) {

	if (pI2Cx == I2C1) {
		I2C1_REG_RESET();
	}
	else if (pI2Cx == I2C2) {
		I2C2_REG_RESET();
	}
	else if (pI2Cx == I2C3) {
		I2C3_REG_RESET();
	}
}


/*
 * Function name:			I2C_PeriClockControl
 *
 * Description:				Enables the I2C peripheral clock
 *
 * Parameter 1:				Base address of the I2C peripheral
 * Parameter 2:				Enable (1) or disable (0)
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {

	if (EnorDi == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}


/*
 * Function name:			I2C_PeripheralControl
 *
 * Description:				Configures the PE bit
 *
 * Parameter 1:				Base address of the I2C peripheral
 * Parameter 2:				Enable or disable
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {

	if (EnOrDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}


/*
 * Function name:			I2C_MasterSendData
 *
 * Description:				Sends data from the master to the slave
 *
 * Parameter 1:				Handler that contains base address of the I2C peripheral and configuration settings
 * Parameter 2:				Pointer to the transmission buffer
 * Parameter 3:				Length of data to be sent
 * Parameter 4:				Address of the slave (7 bits)
 *
 * Return:					None
 *
 * Notes:					This is a blocking call
 *
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t* pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {

	// 1. generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. confirm that start generation is completed by checking the SB flag in the SR1
	//	  (until SB is cleared, the SCL will be stretched)
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. send the address of the slave with r/w bit to w(0)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, WRITE);

	// 4. confirm the address phase is completed by checking the ADDR flag in the SR1
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// 5. clear ADDR flag
	I2C_ClearADDRFlag(pI2CHandle);

	// 6. send the data until Len becomes 0
	while (Len > 0) {
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	// 7. wait until TXE = 1 and BTF = 1 (DR and SR are empty)
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	// 8. generate stop condition
	if (Sr == I2C_DISABLE_SR) {
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}


/*
 * Function name:			I2C_MasterReceiveData
 *
 * Description:				Master receives data from the slave
 *
 * Parameter 1:				Handler that contains base address of the I2C peripheral and configuration settings
 * Parameter 2:				Pointer to the reception buffer
 * Parameter 3:				Length of data to be received
 * Parameter 4:				Address of the slave (7 bits)
 *
 * Return:					None
 *
 * Notes:					This is a blocking call
 *
 */

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {

	// 1. generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. confirm that start generation is completed by checking the SB flag in SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. send the address of the slave with the r/w bit set to R(1) (8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, READ);

	// 4. wait until the address phase is complete by checking the ADDR flag in SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// 5. read data from slave
	if (Len == 1) {		// only 1 byte
		// disable the ACK
		I2C_ManageACK(pI2CHandle->pI2Cx, DISABLE);

		// clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// wait until RXNE = 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		// generate stop condition
		if (Sr == I2C_DISABLE_SR) {
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// read data into buffer
		*pRxbuffer = pI2CHandle->pI2Cx->DR;
	}

	if (Len > 1) { 		// more than 1 byte
		// clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// read data until Len becomes 0
		for (uint32_t i = Len; i > 0; i--) {
			// wait until RXNE = 1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if (i == 2) { 		// generate stop condition at last 2 bytes
				// disable the ACK
				I2C_ManageACK(pI2CHandle->pI2Cx, DISABLE);

				// generate stop condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			// read data into buffer and increment address
			*pRxbuffer = pI2CHandle->pI2Cx->DR;
			pRxbuffer++;
		}
	}

	// re-enable ACKing
	if (pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE) {
		I2C_ManageACK(pI2CHandle->pI2Cx, ENABLE);
	}

}

/*
 * Function name:			I2C_MasterSendDataIT
 *
 * Description:				Master sends data to the slave via interrupt
 *
 * Parameter 1:				Handler that contains base address of the I2C peripheral and configuration settings
 * Parameter 2:				Pointer to the transmission buffer
 * Parameter 3:				Length of data to be transmitted
 * Parameter 4:				Address of the slave (7 bits)
 *
 * Return:					State of the peripheral bus
 *
 * Notes:					This is a non-blocking call
 *
 */


uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {

	uint8_t busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {

		pI2CHandle->pTxbuffer = pTxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// enable the ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// enable the ITEVFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// enable the ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}

	return busystate;
}


/*
 * Function name:			I2C_MasterReceiveDataIT
 *
 * Description:				Master receives data from the slave via interrupt
 *
 * Parameter 1:				Handler that contains base address of the I2C peripheral and configuration settings
 * Parameter 2:				Pointer to the reception buffer
 * Parameter 3:				Length of data to be received
 * Parameter 4:				Address of the slave (7 bits)
 *
 * Return:					None
 *
 * Notes:					This is a non-blocking call
 *
 */

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t* pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {

	uint8_t busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {

		pI2CHandle->pRxbuffer = pRxbuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->RxSize = Len;
		pI2CHandle->Sr = Sr;

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// enable the ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// enable the ITEVFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// enable the ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}

	return busystate;
}


/*
 * Function name:			I2C_SlaveSendData
 *
 * Description:				Slave sends data to the master
 *
 * Parameter 1:				Base address of the I2C peripheral
 * Parameter 2:				Data to be sent
 *
 * Return:					None
 *
 * Notes:					This is a non-blocking call
 *
 */

void I2C_SlaveSendData(I2C_RegDef_t* pI2C, uint8_t data) {

	pI2C->DR = data;

}


/*
 * Function name:			I2C_SlaveReceiveData
 *
 * Description:				Slave receives data from the master
 *
 * Parameter 1:				Base address of the I2C peripheral
 * Parameter 2:				Pointer to the reception buffer
 *
 * Return:					None
 *
 * Notes:					This is a non-blocking call
 *
 */

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t* pI2C) {

	return (uint8_t) pI2C->DR;

}



/*
 * Function name:			I2C_GetFlagStatus
 *
 * Description:				Checks the status of the provided flag in the I2Cx SR register
 *
 * Parameter 1:				Base address of the I2Cx peripheral
 * Parameter 2:				Name of flag to be checked
 *
 * Return:					1 or 0
 *
 * Notes:					None
 *
 */

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName) {

	if ((pI2Cx->SR1 & FlagName) || (pI2Cx->SR2 & FlagName)) {
		return FLAG_SET;
	}
	return FLAG_RESET;

}


/*
 * Function name:			I2C_IRQInterruptConfig
 *
 * Description:				Configures the appropriate interrupt register in the NVIC
 *
 * Parameter 1:				IRQ Number
 * Parameter 2:				Enable or Disable (1 or 0)
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {

	if (EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			// program ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);

		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// program ISER1
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			// program ISER2
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));

		}
	} else {
		if (IRQNumber <= 31) {
			// program ICER0
			*NVIC_ICER0 |= (1 << IRQNumber);

		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// program ICER1
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));

		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			// program ICER2
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}
	}
}


/*
 * Function name:			I2C_IRQPriorityConfig
 *
 * Description:				Configures the priority of the interrupt through the priority register in the NVIC
 *
 * Parameter 1:				IRQ Number
 * Parameter 2:				IRQ Priority
 *
 * Return:					None
 *
 * Notes:					NO_PR_BITS_IMPLEMENTED is MCU specific
 *
 */

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {

	// find IPR register and bit to set
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint32_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);	// first 4 bits of each priority register are not used

	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}


void I2C_ManageACK(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {

	if (EnorDi == 1) { 		// enable
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	} else { 				// disable
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}


void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle) {

	// disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// disable ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxbuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	I2C_ManageACK(pI2CHandle->pI2Cx, ENABLE);
}


void I2C_CloseSendData(I2C_Handle_t *pI2CHandle) {

	// disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// disable ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxbuffer = NULL;
	pI2CHandle->TxLen = 0;
}


static void I2C_MasterHandleTXNEInterrupt(I2C_Handle_t *pI2CHandle) {

	if (pI2CHandle->TxLen > 0) {
		// load data into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxbuffer);

		// decrement the TxLen
		(pI2CHandle->TxLen)--;

		// increment the buffer address
		(pI2CHandle->pTxbuffer)++;
	}

}


static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle) {

	// implement data reception
	if (pI2CHandle->RxSize == 1) {
		*(pI2CHandle->pRxbuffer) = pI2CHandle->pI2Cx->DR;

		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxSize > 1) {
		if (pI2CHandle->RxLen == 2) {
			// disable the ACK bit
			I2C_ManageACK(pI2CHandle->pI2Cx, DISABLE);
		}

		// read the DR
		*(pI2CHandle->pRxbuffer) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxbuffer++;
		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxSize == 0) {
		// close the I2C data reception
		if (pI2CHandle->Sr == I2C_DISABLE_SR) {
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// close I2C Rx
		I2C_CloseReceiveData(pI2CHandle);

		// notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_COMPLETE);
	}

}


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {

	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	// 1. SB event (only in master mode)
	if (temp1 && temp3) { 		// SB flag is set
		// execute address phase
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, WRITE);
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, READ);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	// 2. ADDR event
	if (temp1 && temp3) { 		// ADDR flag is set
		// clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	// 3. BTF (byte transfer finished) event
	if (temp1 && temp3) { 		// BTF flag is set
		// check if TXE flag is set
		if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)) {
			// both BTF and TXE are set

			if (pI2CHandle->TxLen == 0) {
				// 1. generate stop condition (check for repeated start)
				if (pI2CHandle->Sr == I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

				// 2. rest all the member elements of the handle structure
				I2C_CloseSendData(pI2CHandle);

				// 3. notify the application that the transmission is complete
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_COMPLETE);
			}
		} else if (pI2CHandle->pI2Cx->SR1 == I2C_BUSY_IN_RX) {
			// nothing to do
			;
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	// 4. STOPF event
	if (temp1 && temp3) { 		// STOPF flag is set
		// clear STOPF by reading SR1 (done above) and writing to CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		// notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	// 5. TXE event
	if (temp1 && temp2 && temp3) { 		// TXE flag is set
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) { 		// check for device mode
			// implement data transmission
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
				I2C_MasterHandleTXNEInterrupt(pI2CHandle);
			}
		} else { 		// slave mode
			// check to see if device is in transmitter mode
			if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)) {
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);

			}
		}
 	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	// 6. RXNE event
	if (temp1 && temp2 && temp3) { 		// RXNE flag is set
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
			// device is in master mode
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		} else { 		// slave mode
			// check that device is in receiver mode
			if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)) {
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);

			}
		}
	}

}


void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle) {

	uint32_t temp1, temp2;

	// check status of the ITERREN control bit
	temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

	// check for Bus error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);

	if (temp1 && temp2) {
		// clear the bus error
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		// notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	// check for arbitration error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);

	if (temp1 && temp2) {
		// clear the arbitration loss error
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		// notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	// check for ACK failure error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);

	if (temp1 && temp2) {
		// clear the ACK failure error
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		// notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	// check for overrun error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);

	if (temp1 && temp2) {
		// clear the overrun error
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		// notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	// check for timeout error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);

	if (temp1 && temp2) {
		// clear the timeout error
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		// notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}


__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent) {

	// this is a weak implementation and the application may override this function.

}


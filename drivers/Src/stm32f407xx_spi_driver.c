/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Aug 26, 2025
 *      Author: himethaluthwala
 */

#include "stm32f407xx_spi_driver.h"


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);


/*
 * Function name:			SPI_Init
 *
 * Description:				Initialises the SPI peripheral
 *
 * Parameter 1:				Handler that contains base address of the SPI peripheral and configuration settings
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void SPI_Init(SPI_Handle_t *pSPIHandle) {

	// configure the SPI_CR1 register
	uint32_t tempreg = 0;

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// configure the device mode
	tempreg |= (pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR);

	// configure bus config
	if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		// bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPI_Config.SPI_DeviceMode == SPI_BUS_CONFIG_HD) {
		// bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPI_Config.SPI_DeviceMode == SPI_BUS_CONFIG_S_RXONLY) {
		// bidi mode should be cleared and RXONLY bit must be set
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// configure the SPI serial clock speed (baud rate)
	tempreg |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR);

	// configure the DFF
	tempreg |= (pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	// configure the CPOL
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	// configure the CPHA
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	// configure the SSM
	tempreg |= (pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 |= tempreg;
}


/*
 * Function name:			SPI_DeInit
 *
 * Description:				Deinitialises the SPI peripheral
 *
 * Parameter 1:				Base address of the SPI peripheral
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void SPI_DeInit(SPI_RegDef_t *pSPIx) {

	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	}
}


/*
 * Function name:			SPI_PeriClockControl
 *
 * Description:				Enables the SPI peripheral clock
 *
 * Parameter 1:				Base address of the SPI peripheral
 * Parameter 2:				Enable (1) or disable (0)
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {

	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}


/*
 * Function name:			SPI_GetFlagStatus
 *
 * Description:				Checks the status of the provided flag in the SPIx SR register
 *
 * Parameter 1:				Base address of the SPI peripheral
 * Parameter 2:				Name of flag to be checked
 *
 * Return:					1 or 0
 *
 * Notes:					None
 *
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {

	if (pSPIx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*
 * Function name:			SPI_SendData
 *
 * Description:				Sends data via the Tx buffer
 *
 * Parameter 1:				Base address of the SPI peripheral
 * Parameter 2:				Pointer to Tx buffer
 * Parameter 3:				Length of data to be sent
 *
 * Return:					None
 *
 * Notes:					This is a blocking call.
 *
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {

	while (Len > 0) {
		// 1. wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2. check the DFF
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			// 16 bit DFF
			// load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;		// twice since 2 bytes of data were sent out.
			(uint16_t*)pTxBuffer++;		// increment to get next byte of data.
		} else {
			// 8 bit DFF
			// load the data into the DR
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}


/*
 * Function name:			SPI_ReceiveData
 *
 * Description:				Receives data via the Rx buffer
 *
 * Parameter 1:				Base address of the SPI peripheral
 * Parameter 2:				Pointer to Rx buffer
 * Parameter 3:				Length of data to be received
 *
 * Return:					None
 *
 * Notes:					This is a blocking call.
 *
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {

	while (Len > 0) {
		// 1. wait until RXNE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		// 2. check the DFF
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			// 16 bit DFF
			// load the data from the DR into the Rxbuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len -= 2;		// twice since 2 bytes of data were sent out.
			(uint16_t*)pRxBuffer++;		// increment to get next byte of data.
		} else {
			// 8 bit DFF
			// load the data from the DR into the Rxbuffer address
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}


/*
 * Function name:			SPI_IRQInterruptConfig
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

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {

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
 * Function name:			SPI_IRQPriorityConfig
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {

	// find IPR register and bit to set
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint32_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);	// first 4 bits of each priority register are not used

	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}


/*
 * Function name:			SPI_IRQHandling
 *
 * Description:				Clears the EXTI pending register if currently set
 *
 * Parameter 1:				Pointer to the SPIx handle structure
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {

	uint8_t temp1, temp2;

	// 1. check for TXE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2) {
		// handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	// 2. check for RXNE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2) {
		// handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	// 3. check for Overrun Flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2) {
		// handle overrun error
		spi_ovr_interrupt_handle(pSPIHandle);
	}

}


/*
 * Function name:			SPI_PeripheralControl
 *
 * Description:				Configures the SPE bit
 *
 * Parameter 1:				Base address of the SPI peripheral
 * Parameter 2:				Enable or disable
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {

	if (EnOrDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/*
 * Function name:			SPI_SSIConfig
 *
 * Description:				Configures the SSOE bit
 *
 * Parameter 1:				Base address of the SPI peripheral
 * Parameter 2:				Enable or disable
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {

	if (EnOrDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


/*
 * Function name:			SPI_SSOEConfig
 *
 * Description:				Configures the SSOE bit
 *
 * Parameter 1:				Base address of the SPI peripheral
 * Parameter 2:				Enable or disable
 *
 * Return:					None
 *
 * Notes:					During hardware slave management, making SSOE = 1 enables the NSS output.
 *
 */

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {

	if (EnOrDi == ENABLE) {
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


/*
 * Function name:			SPI_SendDataIT
 *
 * Description:				Sends data via the Tx buffer through an interrupt.
 *
 * Parameter 1:				Pointer to the SPIx handle structure
 * Parameter 2:				Pointer to Tx buffer
 * Parameter 3:				Length of data to be sent
 *
 * Return:					None
 *
 * Notes:					This is a non-blocking call.
 *
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len) {

	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX) {
		// 1. Save the Tx buffer address and Len information in some global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		// 2. Mark the SPI state as busy in transmission so that no other code can take over the same SPI peripheral
		//    until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit to get an interrupt whenever the TXE flag is set in the SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		// 4. Data transmission will be handled by the ISR code

	}

	return state;
}


/*
 * Function name:			SPI_ReceiveDataIT
 *
 * Description:				Receives data via the Rx buffer
 *
 * Parameter 1:				Pointer to the SPIx handle structure
 * Parameter 2:				Pointer to Rx buffer
 * Parameter 3:				Length of data to be received
 *
 * Return:					None
 *
 * Notes:					This is a non-blocking call.
 *
 */

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len) {

	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX) {
		// 1. Save the Rx buffer address and Len information in some global variable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		// 2. Mark the SPI state as busy in transmission so that no other code can take over the same SPI peripheral
		//    until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. Enable the TXEIE control bit to get an interrupt whenever the RXNE flag is set in the SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		// 4. Data transmission will be handled by the ISR code

	}

	return state;
}


// helper functions

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {

	// check the DFF
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		// 16 bit DFF
		// load the data into the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;		// twice since 2 bytes of data were sent out.
		(uint16_t*)pSPIHandle->pTxBuffer++;		// increment to get next byte of data.
	} else {
		// 8 bit DFF
		// load the data into the DR
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if (!pSPIHandle->TxLen) { 		// length reaches 0
		// close SPI transmission
		// inform the application that Tx is finished
		SPI_CloseTransmission(pSPIHandle);

		// callback function
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle) {

	// check the DFF
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		// 16 bit DFF
		// load the data into the Rx buffer
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;		// twice since 2 bytes of data were received.
		(uint16_t*)pSPIHandle->pRxBuffer++; 	// increment to get next byte of data.
	} else {
		// 8 bit DFF
		// load the data into the Rx buffer
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++; 	// increment to get next byte of data.
	}

	if (!pSPIHandle->RxLen) { 		// length reaches 0 (reception is complete)
		// close SPI transmission
		// inform the application that Rx is finished
		SPI_CloseReception(pSPIHandle);

		// callback function
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle) {

	uint8_t temp;

	// 1. clear the ovr flag
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX) {
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;

	// 2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {

	uint8_t temp;

	temp = pSPIx->DR;
	temp = pSPIx->SR;

	(void)temp;
}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle) {

	// inform the application that Tx is finished
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);		// prevents interrupts from TXE flag

	// reset buffers
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}


void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {

	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);		// prevents interrupts from RXNE flag

	// reset buffers
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}


__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent) {

	// this is a weak implementation and the application may override this function.

}


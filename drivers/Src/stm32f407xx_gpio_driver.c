/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Aug 2, 2025
 *      Author: himethaluthwala
 */


#include "stm32f407xx_gpio_driver.h"


/*
 * Function name:			GPIO_Init
 *
 * Description:				Initialises the GPIO port and pin
 *
 * Parameter 1:				Handler that contains base address of the GPIO peripheral and the pin configuration settings
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	uint32_t temp = 0;

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. configure GPIO pin mode
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)		// non-interrupt mode
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//clearing bit fields
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else		// interrupt mode
	{
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT) {
			// configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT) {
			// configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT) {
			// configure both registers
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// configure GPIO Port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;		// index for EXTI control register
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;		// offset within register
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);		// GPIO port code

		SYSCFG_PCLK_EN();		// enable SYSCFG peripheral clock
		SYSCFG->EXTICR[temp1] |= (portcode << (temp2 * 4));

		// enable EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	temp = 0;

	//2. configure pin output speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//3. configure the pull-up / pull-down settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//5. configure the alternate functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		// configure the alternate function mode
		uint8_t temp3, temp4;

		temp3 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;		// alternate function low or high register
		temp4 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;		// offset in the register
		pGPIOHandle->pGPIOx->AFR[temp3] &= ~(0xF << (4 * temp4));
		pGPIOHandle->pGPIOx->AFR[temp3] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp4));
	}

}


/*
 * Function name:			GPIO_DeInit
 *
 * Description:				Deinitialises the GPIO port and pin
 *
 * Parameter 1:				Base address of the GPIO peripheral
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

/*
 * Function name:			GPIO_PeriClockControl
 *
 * Description:				Enables or disables the peripheral clock for the given GPIO port
 *
 * Parameter 1:				Base address of the GPIO peripheral
 * Parameter 2:				ENABLE or DISABLE macros
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{

	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}

	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}


/*
 * Function name:			GPIO_ReadFromInputPin
 *
 * Description:				Reads the value at the given GPIO pin
 *
 * Parameter 1:				Base address of the GPIO peripheral
 * Parameter 2:				GPIO pin number
 *
 * Return:					0 or 1
 *
 * Notes:					None
 *
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);		//shift the desired bit to the first position and mask all other bits

	return value;
}


/*
 * Function name:			GPIO_ReadFromInputPort
 *
 * Description:				Reads the values at the entire GPIO port
 *
 * Parameter 1:				Base address of the GPIO peripheral
 *
 * Return:					16 bits
 *
 * Notes:					None
 *
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
}


/*
 * Function name:			GPIO_WriteToOuputPin
 *
 * Description:				Writes 1 or 0 to the specified pin of the GPIO port output data register
 *
 * Parameter 1:				Base address of the GPIO peripheral
 * Parameter 2:				GPIO pin number
 * Parameter 3:				Value to write (1 or 0)
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void GPIO_WriteToOuputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == 1)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}


/*
 * Function name:			GPIO_WriteToOuputPort
 *
 * Description:				Writes a value to the specified GPIO port output register
 *
 * Parameter 1:				Base address of the GPIO peripheral
 * Parameter 2:				Value to write
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void GPIO_WriteToOuputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->IDR = Value;
}


/*
 * Function name:			GPIO_ToggleOuputPin
 *
 * Description:				Toggles the specified GPIO output pin via the output data register
 *
 * Parameter 1:				Base address of the GPIO peripheral
 * Parameter 2:				GPIO pin number
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void GPIO_ToggleOuputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*
 * Function name:			GPIO_IRQInterruptConfig
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

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
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
 * Function name:			GPIO_IRQPriorityConfig
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {
	// find IPR register and bit to set
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint32_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);	// first 4 bits of each priority register are not used

	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}


/*
 * Function name:			GPIO_IRQHandling
 *
 * Description:				Clears the EXTI pending register if currently set
 *
 * Parameter 1:				GPIO pin number
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear the EXTI pending register
	if (EXTI->PR & (1 << PinNumber)) {
		// clear by writing 1
		EXTI->PR |= (1 << PinNumber);
	}
}


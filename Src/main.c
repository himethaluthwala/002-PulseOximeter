/*
 * main.c
 *
 *  Created on: Oct 6, 2025
 *      Author: himethaluthwala
 */


#include <ssd1306_driver.h>


SSD1306_Handle_t SSD1306Handle;

/*
 * SSD1306 / SPI2 Pins
 * SCLK			PB13
 * SDIN			PB15
 * CS			PB12
 * Alternate function mode 5
 */

void SPI2_GPIO_Inits(void) {

	// clear memory values of structures
	memset(&SSD1306Handle.SCLK_Handle, 0, sizeof(SSD1306Handle.SCLK_Handle));
	memset(&SSD1306Handle.SDIN_Handle, 0, sizeof(SSD1306Handle.SDIN_Handle));
	//memset(&SSD1306Handle.CS_Handle, 0, sizeof(SSD1306Handle.CS_Handle));

	// SCLK
	SSD1306Handle.SCLK_Handle.pGPIOx = GPIOB;
	SSD1306Handle.SCLK_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	SSD1306Handle.SCLK_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SSD1306Handle.SCLK_Handle.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SSD1306Handle.SCLK_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SSD1306Handle.SCLK_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SSD1306Handle.SCLK_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&SSD1306Handle.SCLK_Handle);

	// MOSI / SDIN
	SSD1306Handle.SDIN_Handle.pGPIOx = GPIOB;
	SSD1306Handle.SDIN_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	SSD1306Handle.SDIN_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SSD1306Handle.SDIN_Handle.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SSD1306Handle.SDIN_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SSD1306Handle.SDIN_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SSD1306Handle.SDIN_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&SSD1306Handle.SDIN_Handle);

	// NSS / CS
/*	SSD1306Handle.CS_Handle.pGPIOx = GPIOB;
	SSD1306Handle.CS_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	SSD1306Handle.CS_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SSD1306Handle.CS_Handle.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SSD1306Handle.CS_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SSD1306Handle.CS_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SSD1306Handle.CS_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&SSD1306Handle.CS_Handle);
*/
}


void SPI2_Inits(void) {

	// clear memory values of structures
	memset(&SSD1306Handle.SPIHandle, 0, sizeof(SSD1306Handle.SPIHandle));

	SSD1306Handle.SPIHandle.pSPIx = SPI2;
	SSD1306Handle.SPIHandle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SSD1306Handle.SPIHandle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;		// full duplex mode
	SSD1306Handle.SPIHandle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SSD1306Handle.SPIHandle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV4;		// 4 MHz
	SSD1306Handle.SPIHandle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SSD1306Handle.SPIHandle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SSD1306Handle.SPIHandle.SPI_Config.SPI_SSM = SPI_SSM_EN;		// software slave management enabled for NSS pin;

	SPI_Init(&SSD1306Handle.SPIHandle);

}


/*
 * RESET  	PC5
 * DC  		PC0
 * CS		PB12
 */
void SSD1306_GPIO_Inits(void) {

	// clear memory values of structures
	memset(&SSD1306Handle.RESET_Handle, 0, sizeof(SSD1306Handle.RESET_Handle));
	memset(&SSD1306Handle.DC_Handle, 0, sizeof(SSD1306Handle.DC_Handle));
	memset(&SSD1306Handle.CS_Handle, 0, sizeof(SSD1306Handle.CS_Handle));

	SSD1306Handle.RESET_Handle.pGPIOx = GPIOC;
	SSD1306Handle.RESET_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	SSD1306Handle.RESET_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	SSD1306Handle.RESET_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SSD1306Handle.RESET_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SSD1306Handle.RESET_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&SSD1306Handle.RESET_Handle);

	SSD1306Handle.DC_Handle.pGPIOx = GPIOC;
	SSD1306Handle.DC_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	SSD1306Handle.DC_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	SSD1306Handle.DC_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SSD1306Handle.DC_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SSD1306Handle.DC_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&SSD1306Handle.DC_Handle);

	SSD1306Handle.CS_Handle.pGPIOx = GPIOB;
	SSD1306Handle.CS_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	SSD1306Handle.CS_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	SSD1306Handle.CS_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SSD1306Handle.CS_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SSD1306Handle.CS_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&SSD1306Handle.CS_Handle);

}


void UserButton_GPIO_Inits(void) {

	GPIO_Handle_t GPIO_Btn;

	// clear memory values of structures
	memset(&GPIO_Btn, 0, sizeof(GPIO_Btn));

	// User button connected to PA0
	GPIO_Btn.pGPIOx = GPIOA;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;		// rising edge trigger
	GPIO_Btn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&GPIO_Btn);

	// Button IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

}


int main(void) {

	uint8_t value = 98;

	// initialise SPI2 peripheral
	SPI2_GPIO_Inits();
	SPI2_Inits();
	SPI_SSIConfig(SPI2, ENABLE); // SSI bit to 1 to avoid mode fault with the NSS during software slave management
	SPI_PeripheralControl(SPI2, ENABLE); // enable the SPI2 peripheral before data is sent

	// initialise the OLED screen
	SSD1306_GPIO_Inits();
	SSD1306_Init(&SSD1306Handle);

	// initialise the user button
	UserButton_GPIO_Inits();

	// write a value into the buffer and display it on the screen
	delay(100);
	SSD1306_DisplayValue(&SSD1306Handle, value);

	return 0;
}


void EXTI0_IRQHandler(void) {

	delay(200);	// 200ms delay to avoid button de-bouncing
	GPIO_IRQHandling(GPIO_PIN_NO_0);

	// power down display
	SSD1306_PowerDown(&SSD1306Handle);

}

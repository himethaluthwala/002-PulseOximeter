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
 * DC			PC0
 * Alternate function mode 5
 */

void SPI2_GPIOInits(void) {

	// SCLK
	SSD1306Handle.SCLK_Handle.pGPIOx = GPIOB;
	SSD1306Handle.SCLK_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	SSD1306Handle.SCLK_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SSD1306Handle.SCLK_Handle.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SSD1306Handle.SCLK_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SSD1306Handle.SCLK_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SSD1306Handle.SCLK_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&SSD1306Handle.SCLK_Handle);

	// MOSI
	SSD1306Handle.SDIN_Handle.pGPIOx = GPIOB;
	SSD1306Handle.SDIN_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	SSD1306Handle.SDIN_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SSD1306Handle.SDIN_Handle.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SSD1306Handle.SDIN_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SSD1306Handle.SDIN_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SSD1306Handle.SDIN_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&SSD1306Handle.SDIN_Handle);

	// NSS
	SSD1306Handle.CS_Handle.pGPIOx = GPIOB;
	SSD1306Handle.CS_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	SSD1306Handle.CS_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SSD1306Handle.CS_Handle.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SSD1306Handle.CS_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SSD1306Handle.CS_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SSD1306Handle.CS_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&SSD1306Handle.CS_Handle);

}


void SPI2_Inits(void) {

	SSD1306Handle.SPIHandle.pSPIx = SPI2;
	SSD1306Handle.SPIHandle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SSD1306Handle.SPIHandle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_HD;		// half duplex mode
	SSD1306Handle.SPIHandle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SSD1306Handle.SPIHandle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;		// 8 MHz
	SSD1306Handle.SPIHandle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SSD1306Handle.SPIHandle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SSD1306Handle.SPIHandle.SPI_Config.SPI_SSM = SPI_SSM_EN; 		// software slave management enabled for NSS pin;

	SPI_Init(&SSD1306Handle.SPIHandle);

}


void SSD1306_DC_Inits(void) {

	SSD1306Handle.DC_Handle.pGPIOx = GPIOC;
	SSD1306Handle.DC_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	SSD1306Handle.DC_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	SSD1306Handle.DC_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SSD1306Handle.DC_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SSD1306Handle.DC_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&SSD1306Handle.DC_Handle);

}


int main(void) {

	//uint8_t data[] = {0x20};

	// initialise SPI2 GPIOs
	SPI2_GPIOInits();

	// initialise SPI2 peripheral
	SPI2_Inits();

	// initialise SSD1306 DC pin
	SSD1306_DC_Inits();

	// sets SSI bit to 1 to avoid error with the NSS during software slave management
	SPI_SSIConfig(SPI2, ENABLE);

	// enable the SPI2 peripheral before data is sent
	SPI_PeripheralControl(SPI2, ENABLE);

	// initialise the OLED screen
	SSD1306_Init(&SSD1306Handle);

	// write some info to the screen

	// update screen
	SSD1306_UpdateScreen(&SSD1306Handle);

	return 0;

}

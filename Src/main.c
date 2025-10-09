/*
 * main.c
 *
 *  Created on: Oct 6, 2025
 *      Author: himethaluthwala
 */


#include <ssd1306_driver.h>


SSD1306_Handle_t SSD1306Handle;


/*
 * SSD1306 Pins
 * SCLK			PB13
 * SDIN			PB15
 * CS			PB12
 * DC			PC0
 */

GPIO_Handle_t SSD1306_DC_Pin;


void SSD1306_GPIO_Inits(void) {

	SSD1306_DC_Pin.pGPIOx = GPIOC;
	SSD1306_DC_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	SSD1306_DC_Pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	SSD1306_DC_Pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SSD1306_DC_Pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&SSD1306_DC_Pin);
}


/*
 * PB12	--> NSS
 * PB13 -->	SCK
 * PB15	-->	MOSI
 * Alternate function mode 5
 */

void SPI2_GPIOInits(void) {

	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	// NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&SPIPins);

}


void SPI2_Inits(void) {

	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_HD;		// half duplex mode
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;		// 8 MHz
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_EN; 		// software slave management enabled for NSS pin;

	SPI_Init(&SPI2Handle);
}


int main(void) {

	uint8_t command;

	// initialise SPI2 GPIOs
	SPI2_GPIOInits();

	// initialise SPI2 peripheral
	SPI2_Inits();

	// sets SSI bit to 1 to avoid error with the NSS during software slave management
	SPI_SSIConfig(SPI2, ENABLE);

	// enable the SPI2 peripheral before data is sent
	SPI_PeripheralControl(SPI2, ENABLE);

	// send command to OLED
	command = 0xAE;
	SSD1306_SendCommand(&SSD1306Handle, command);


	return 0;
}

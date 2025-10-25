/*
 * ssd1306_driver.c
 *
 *  Created on: Oct 6, 2025
 *      Author: himethaluthwala
 */


#include <ssd1306_driver.h>


static void SSD1306_Config(SSD1306_Handle_t *pHandle);

static uint8_t ssd1306_buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

static const uint8_t Init_Seq[] = {

	SET_DISPLAY_OFF,
	DISPLAY_CLK_DIV_RATIO, 0x80,
	SET_MULTIPLEX_RATIO, 0x3F, 		// 63
	SET_CONTRAST_CONTROL, 0x7F,
	SEGMENT_REMAP_DEFAULT,
	COM_OUTPUT_SCAN_DEFAULT,
	COM_HARDWARE_CONFIG, COM_HW_SEQ_DI_LR,
	SET_VCOMH_DESELECT, VCOMH_DEFAULT, 		// ~0.77 x Vcc
	DISPLAY_OFFSET, 0x00,
	SET_START_LINE,
	SET_MEM_ADDR_MODE, 0x00, 		// horizontal
	SET_CHARGE_PUMP, ENABLE_CHARGE_PUMP,
	SET_PRE_CHARGE, PRE_CHARGE_STABLE,
	DISPLAY_NORMAL,
	DISPLAY_RAM,
	SET_DISPLAY_ON

};

static const uint8_t Font16x24[][48] = {

	[0] = {},		// bitmap for 0
	[1] = {},		// bitmap for 1
	[2] = {},
	[3] = {},
	[4] = {},
	[5] = {},
	[6] = {},
	[7] = {},
	[8] = {},
	[9] = {},
	[10] = {},

};


void delay(uint32_t msDelay) { 		// based on 16 MHz MCU clock

	for (uint32_t i = (msDelay*1250); i > 0; i--);

}


/*
 * Function name:		 	SSD1306_Init
 *
 * Description:				Initialises the OLED screen
 *
 * Parameter 1:				Pointer to the handle structure for the peripheral
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void SSD1306_Init(SSD1306_Handle_t *pHandle) {

	// toggle RESET pin
	GPIO_WriteToOuputPin(pHandle->RESET_Handle.pGPIOx, pHandle->RESET_Handle.GPIO_PinConfig.GPIO_PinNumber, LOW);
	delay(10);
	GPIO_WriteToOuputPin(pHandle->RESET_Handle.pGPIOx, pHandle->RESET_Handle.GPIO_PinConfig.GPIO_PinNumber, HIGH);

	// send set-up commands
	delay(100);
	SSD1306_Config(pHandle);

	delay(100);		// stabilisation period

}


/*
 * Function name:		 	SSD1306_Config
 *
 * Description:				Configures the OLED screen by sending the set-up commands
 *
 * Parameter 1:				Pointer to the handle structure for the peripheral
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

static void SSD1306_Config(SSD1306_Handle_t *pHandle) {

	for (uint32_t i = 0; i < sizeof(Init_Seq); i++) {
		SSD1306_SendCommand(pHandle, Init_Seq[i]);
	}

}


/*
 * Function name:		 	SSD1306_DisplayValue
 *
 * Description:				Writes a value to the buffer and updates the GDDRAM
 *
 * Parameter 1:				Pointer to the handle structure for the peripheral
 * Parameter 2:				Value to be displayed
 *
 * Return:					None
 *
 * Notes:					This is a blocking call.
 *
 */

void SSD1306_DisplayValue(SSD1306_Handle_t *pHandle, uint8_t value) {



}


/*
 * Function name:		 	SSD1306_UpdateScreen
 *
 * Description:				Updates the GDDRAM according to the buffer contents
 *
 * Parameter 1:				Pointer to the handle structure for the peripheral
 *
 * Return:					None
 *
 * Notes:					This is a blocking call.
 *
 */

void SSD1306_UpdateScreen(SSD1306_Handle_t *pHandle) {

	// reset column and page address pointers
	SSD1306_SendCommand(pHandle, SET_COL_ADDR);
	SSD1306_SendCommand(pHandle, 0x00);
	SSD1306_SendCommand(pHandle, 0x7F);

	SSD1306_SendCommand(pHandle, SET_PAGE_ADDR);
	SSD1306_SendCommand(pHandle, 0x00);
	SSD1306_SendCommand(pHandle, 0x07);

	// send buffer to the GDDRAM
	SSD1306_SendData(pHandle, ssd1306_buffer, sizeof(ssd1306_buffer));

}


/*
 * Function name:		 	SSD1306_DrawChar
 *
 * Description:				Updates the GDDRAM according to the buffer contents
 *
 * Parameter 1:				Pointer to the handle structure for the peripheral
 * Parameter 2:				Glyph from Font bitmap
 * Parameter 3:				Horizontal coordinate
 * Parameter 4:				Vertical coordinate
 *
 * Return:					None
 *
 * Notes:					None
 *
 */

void SSD1306_DrawChar(SSD1306_Handle_t *pHandle, const uint8_t glyph, uint8_t x, uint8_t y) {



}


/*
 * Function name:		 	SSD1306_SendCommand
 *
 * Description:				Sends command to the OLED display via SPI
 *
 * Parameter 1:				Pointer to the handle structure for the peripheral
 * Parameter 2:				Command to be sent
 *
 * Return:					None
 *
 * Notes:					This is a blocking call.
 *
 */


void SSD1306_SendCommand(SSD1306_Handle_t *pHandle, uint8_t command) {

	// pull DC pin low for command
	GPIO_WriteToOuputPin(pHandle->DC_Handle.pGPIOx, pHandle->DC_Handle.GPIO_PinConfig.GPIO_PinNumber, LOW);

	// pull CS pin low to select OLED as slave
	GPIO_WriteToOuputPin(pHandle->CS_Handle.pGPIOx, pHandle->CS_Handle.GPIO_PinConfig.GPIO_PinNumber, LOW);

	// send command over SPI
	SPI_SendData(pHandle->SPIHandle.pSPIx, &command, 1);

	// pull CS back to high
	GPIO_WriteToOuputPin(pHandle->CS_Handle.pGPIOx, pHandle->CS_Handle.GPIO_PinConfig.GPIO_PinNumber, HIGH);

}


/*
 * Function name:		 	SSD1306_SendData
 *
 * Description:				Sends data to the OLED display via SPI
 *
 * Parameter 1:				Pointer to the handle structure for the peripheral
 * Parameter 2:				Pointer to transmission buffer
 * Parameter 3: 			Length of data in bytes
 *
 * Return:					None
 *
 * Notes:					This is a blocking call.
 *
 */


void SSD1306_SendData(SSD1306_Handle_t *pHandle, uint8_t *pData, uint32_t Len) {

	// pull DC pin high for data
	GPIO_WriteToOuputPin(pHandle->DC_Handle.pGPIOx, pHandle->DC_Handle.GPIO_PinConfig.GPIO_PinNumber, HIGH);

	// pull CS pin low to select OLED as slave
	GPIO_WriteToOuputPin(pHandle->CS_Handle.pGPIOx, pHandle->CS_Handle.GPIO_PinConfig.GPIO_PinNumber, LOW);

	// send data over SPI
	SPI_SendData(pHandle->SPIHandle.pSPIx, pData, Len);

	// pull CS back to high
	GPIO_WriteToOuputPin(pHandle->CS_Handle.pGPIOx, pHandle->CS_Handle.GPIO_PinConfig.GPIO_PinNumber, HIGH);

}





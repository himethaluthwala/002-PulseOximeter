/*
 * ssd1306_driver.c
 *
 *  Created on: Oct 6, 2025
 *      Author: himethaluthwala
 */


#include <ssd1306_driver.h>


static uint8_t ssd1306_buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];


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

void SSD1306_Init(SSD1306_Handle_t *pSSD1306Handle) {

	// set display off
	SSD1306_SendCommand(pSSD1306Handle, SET_DISPLAY_OFF);

	// set normal display
	SSD1306_SendCommand(pSSD1306Handle, DISPLAY_NORMAL);

	// set display frequency
	SSD1306_SendCommand(pSSD1306Handle, DISPLAY_CLK_DIV_RATIO);
	SSD1306_SendCommand(pSSD1306Handle, 0x80);

	// set multiplex ratio
	SSD1306_SendCommand(pSSD1306Handle, SET_MULTIPLEX_RATIO);
	SSD1306_SendCommand(pSSD1306Handle, 0x3F); 		// 63

	// set display offset
	SSD1306_SendCommand(pSSD1306Handle, DISPLAY_OFFSET);
	SSD1306_SendCommand(pSSD1306Handle, 0x00); 		// no offset

	// set memory addressing mode
	SSD1306_SendCommand(pSSD1306Handle, MEM_ADDR_MODE_HORI);

	// set COM pin hardware configuration
	SSD1306_SendCommand(pSSD1306Handle, COM_HARDWARE_CONFIG);

	// set display on
	SSD1306_SendCommand(pSSD1306Handle, SET_DISPLAY_ON);

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


void SSD1306_SendCommand(SSD1306_Handle_t *pSSD1306Handle, uint8_t command) {

	// pull DC pin low for command
	GPIO_WriteToOuputPin(pSSD1306Handle->pDC_port, pSSD1306Handle->DC_pin, LOW);

	// pull CS pin low to select OLED as slave
	GPIO_WriteToOuputPin(pSSD1306Handle->pCS_port, pSSD1306Handle->CS_pin, LOW);

	// send command over SPI
	SPI_SendData(pSSD1306Handle->pSPIHandle->pSPIx, &command, 1);

	// pull CS back to high
	GPIO_WriteToOuputPin(pSSD1306Handle->pCS_port, pSSD1306Handle->CS_pin, HIGH);
}


/*
 * Function name:		 	SSD1306_SendData
 *
 * Description:				Sends data to the OLED display via SPI
 *
 * Parameter 1:				Pointer to the handle structure for the peripheral
 * Parameter 2:				Pointer to transmission buffer
 * Parameter 3: 			Length of data
 *
 * Return:					None
 *
 * Notes:					This is a blocking call.
 *
 */


void SSD1306_SendData(SSD1306_Handle_t *pSSD1306Handle, uint8_t *pData, uint32_t Len) {

	// pull DC pin high for data
	GPIO_WriteToOuputPin(pSSD1306Handle->pDC_port, pSSD1306Handle->DC_pin, HIGH);

	// pull CS pin low to select OLED as slave
	GPIO_WriteToOuputPin(pSSD1306Handle->pCS_port, pSSD1306Handle->CS_pin, LOW);

	// send command over SPI
	SPI_SendData(pSSD1306Handle->pSPIHandle->pSPIx, pData, 1);

	// pull CS back to high
	GPIO_WriteToOuputPin(pSSD1306Handle->pCS_port, pSSD1306Handle->CS_pin, HIGH);
}





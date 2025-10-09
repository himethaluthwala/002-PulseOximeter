/*
 * ssd1306_driver.h
 *
 *  Created on: Oct 6, 2025
 *      Author: himethaluthwala
 */

#ifndef INC_SSD1306_DRIVER_H_
#define INC_SSD1306_DRIVER_H_

#include "stm32f407xx.h"


/********************************************************************************/

/* SSD1306 handle structure */

/********************************************************************************/

typedef struct {

    SPI_Handle_t    *pSPIHandle;
    GPIO_RegDef_t   *pDC_port;
    uint8_t         DC_pin;
    GPIO_RegDef_t   *pCS_port;
    uint8_t         CS_pin;
    GPIO_RegDef_t   *pRESET_port;
    uint8_t         RESET_pin;

}SSD1306_Handle_t;


#define SSD1306_WIDTH  				128
#define SSD1306_HEIGHT 				64


/********************************************************************************/

/* SSD1306 Commands */

/********************************************************************************/

#define SET_DISPLAY_ON				0xAF
#define SET_DISPLAY_OFF				0xAE

#define SET_MULTIPLEX_RATIO			0xA5

#define DISPLAY_NORMAL				0xA6
#define DISPLAY_INVERSE				0xA7

#define ENTIRE_DISPLAY_ON			0xA4
#define ENTIRE_DISPLAY_OFF			0xA5

#define DISPLAY_OFFSET				0xD3

#define DISPLAY_CLK_DIV_RATIO		0xD5

#define COM_HARDWARE_CONFIG			0xDA

#define MEM_ADDR_MODE_HORI			0x20
#define MEM_ADDR_MODE_VERT			0x21
#define MEM_ADDR_MODE_PAGE			0x22



/********************************************************************************/

/* APIs supported by this driver */

/********************************************************************************/

/* Main */

void SSD1306_Init(SSD1306_Handle_t *pSSD1306Handle);
void SSD1306_SendCommand(SSD1306_Handle_t *pSSD1306Handle, uint8_t command);
void SSD1306_SendData(SSD1306_Handle_t *pSSD1306Handle, uint8_t *pData, uint32_t Len);

void SSD1306_DrawString(void);
void SSD1306_UpdateScreen(void);


#endif /* INC_SSD1306_DRIVER_H_ */

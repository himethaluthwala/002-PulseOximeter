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

    SPI_Handle_t    SPIHandle;
    GPIO_Handle_t	SCLK_Handle;
    GPIO_Handle_t	SDIN_Handle;
    GPIO_Handle_t   RESET_Handle;
    GPIO_Handle_t   DC_Handle;
    GPIO_Handle_t   CS_Handle;

}SSD1306_Handle_t;


#define SSD1306_WIDTH  				128
#define SSD1306_HEIGHT 				64


/********************************************************************************/

/* SSD1306 Commands */

/********************************************************************************/

#define SET_DISPLAY_ON				0xAF
#define SET_DISPLAY_OFF				0xAE

#define SET_CONTRAST_CONTROL		0x81

#define SEGMENT_REMAP_DEFAULT		0xA0
#define SEGMENT_REMAP_MIRROR		0xA1

#define SET_MULTIPLEX_RATIO			0xA8

#define DISPLAY_NORMAL				0xA6
#define DISPLAY_INVERSE				0xA7

#define DISPLAY_RAM					0xA4
#define ENTIRE_DISPLAY_ON			0xA5

#define DISPLAY_OFFSET				0xD3

#define SET_START_LINE				0x40

#define COM_OUTPUT_SCAN_DEFAULT		0xC0

#define DISPLAY_CLK_DIV_RATIO		0xD5

#define COM_HARDWARE_CONFIG			0xDA
#define COM_HW_SEQ_DI_LR			0x02
#define COM_HW_SEQ_EN_LR			0x22
#define COM_HW_ALT_EN_LR			0x32
#define COM_HW_ALT_DI_LR			0x12

#define SET_VCOMH_DESELECT			0xDB
#define VCHOMH_LOW					0x00
#define VCOMH_DEFAULT				0x20
#define VCOMH_HIGH					0x30
¬
#define SET_CHARGE_PUMP				0x8D
#define ENABLE_CHARGE_PUMP			0x14
#define DISABLE_CHARGE_PUMP			0x10

#define SET_PRE_CHARGE				0xD9
#define PRE_CHARGE_DEFAULT			0x22
#define PRE_CHARGE_STABLE			0xF1

#define SET_MEM_ADDR_MODE			0x20
#define SET_COL_ADDR				0x21
#define SET_PAGE_ADDR				0x22


/********************************************************************************/

/* APIs supported by this driver */

/********************************************************************************/

/* Main */

void SSD1306_Init(SSD1306_Handle_t *pHandle);

void SSD1306_UpdateScreen(SSD1306_Handle_t *pHandle);
void SSD1306_DisplayValue(SSD1306_Handle_t *pHandle, uint8_t value);

void SSD1306_SendCommand(SSD1306_Handle_t *pHandle, uint8_t command);
void SSD1306_SendData(SSD1306_Handle_t *pHandle, uint8_t *pData, uint32_t Len);



#endif /* INC_SSD1306_DRIVER_H_ */

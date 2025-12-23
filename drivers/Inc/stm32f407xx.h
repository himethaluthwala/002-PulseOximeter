/*
 * stm32f407xx.h
 *
 *  Created on: Jul 17, 2025
 *      Author: himethaluthwala
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_


#include <stdint.h>
#include <stddef.h>
#include <string.h>
//#include "core_cm4.h"


/********************************************************************************/

/* PROCESSOR SPECIFIC DETAILS - ARM CORTEX M4 */

/********************************************************************************/

/* NVIC ISERx (Interrupt set-enable register) base addresses */

#define NVIC_ISER0								((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1								((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2								((volatile uint32_t*)0xE000E108)


/* NVIC ICERx (Interrupt clear-enable register) base addresses */

#define NVIC_ICER0								((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1								((volatile uint32_t*)0XE000E184)
#define NVIC_ICER2								((volatile uint32_t*)0XE000E188)


/* NVIC IPR (Interrupt priority register) base address */

#define NVIC_PR_BASEADDR						((volatile uint32_t*)0xE000E400)


/* Number of priority bits implemented in priority register */

#define NO_PR_BITS_IMPLEMENTED					4


/********************************************************************************/

/* MCU Peripheral Base Addresses */

/********************************************************************************/

/* Base addresses of Flash, SRAM and ROM memories */

#define FLASH_BASEADDR							0x08000000U
#define SRAM1_BASEADDR							0x20000000U
#define SRAM2_BASEADDR							0x2001C000U
#define ROM_BASEADDR							0x1FFF0000U


/* Base addresses of APBx and AHBx peripheral buses */

#define APB1_BASEADDR							0x40000000U
#define APB2_BASEADDR							0x40010000U
#define AHB1_BASEADDR							0x40020000U
#define AHB2_BASEADDR							0x50000000U


/* Base addresses of AHB1 peripherals */

#define GPIOA_BASEADDR							(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR							(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR							(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR							(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR							(AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR							(AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR							(AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR							(AHB1_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR							(AHB1_BASEADDR + 0x2000)

#define RCC_BASEADDR							(AHB1_BASEADDR + 0x3800)


/* Base addresses of APB1 peripherals (I2C, SPI, UART and USART only) */

#define I2C1_BASEADDR							(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR							(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR							(APB1_BASEADDR + 0x5C00)
#define SPI2_BASEADDR							(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR							(APB1_BASEADDR + 0x3C00)
#define UART4_BASEADDR							(APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR							(APB1_BASEADDR + 0x5000)
#define USART2_BASEADDR							(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR							(APB1_BASEADDR + 0x4800)


/* Base addresses of APB2 peripherals (SPI, USART, ADC, EXTI and SYSCFG only) */

#define SPI1_BASEADDR							(APB2_BASEADDR + 0x3000)
#define USART1_BASEADDR							(APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR							(APB2_BASEADDR + 0x1400)
#define ADC1_BASEADDR							(APB2_BASEADDR + 0x2000)
#define ADC_COMMON_BASEADDR						(APB2_BASEADDR + 0x2300)
#define ADC2_BASEADDR							(APB2_BASEADDR + 0x2100)
#define ADC3_BASEADDR							(APB2_BASEADDR + 0x2200)
#define EXTI_BASEADDR							(APB2_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR							(APB2_BASEADDR + 0x3800)


/********************************************************************************/

/* Peripheral register definition structures */

/********************************************************************************/


/* GPIO */

typedef struct
{
	volatile uint32_t MODER;					/* GPIO port mode register						Offset: 0x00 */
	volatile uint32_t OTYPER;					/* GPIO port output type register 				Offset: 0x04 */
	volatile uint32_t OSPEEDR;					/* GPIO port output speed register 				Offset: 0x08 */
	volatile uint32_t PUPDR;					/* GPIO port output pullup/pulldown register 	Offset: 0x0C */
	volatile uint32_t IDR;						/* GPIO port input data register 				Offset: 0x10 */
	volatile uint32_t ODR;						/* GPIO port output data register 				Offset: 0x14 */
	volatile uint32_t BSRR;						/* GPIO port bit set/reset register 			Offset: 0x18 */
	volatile uint32_t LCKR;						/* GPIO port configuration lock register 		Offset: 0x1C */
	volatile uint32_t AFR[2];					/* GPIO port alternate function register	 	Offset: (low)0x20, (high)0x24 */
} GPIO_RegDef_t;


/* SPI */

typedef struct
{
	volatile uint32_t CR1;						/* SPI control register 1						Offset: 0x00 */
	volatile uint32_t CR2;						/* SPI control register 2						Offset: 0x04 */
	volatile uint32_t SR;						/* SPI status register							Offset: 0x08 */
	volatile uint32_t DR;						/* SPI data register						 	Offset: 0x0C */
	volatile uint32_t CRCPR;					/* SPI CRC polynomial register					Offset: 0x10 */
	volatile uint32_t RXCRCR;					/* SPI Rx CRC register							Offset: 0x14 */
	volatile uint32_t TXCRCR;					/* SPI Tx CRC register							Offset: 0x18 */
	volatile uint32_t I2SCFGR;					/* SPI I2S configuration register				Offset: 0x1C */
	volatile uint32_t I2SPR;					/* SPI I2S prescalar register				 	Offset: 0x20 */
} SPI_RegDef_t;


/* I2C */

typedef struct
{
	volatile uint32_t CR1;						/* I2C control register 1						Offset: 0x00 */
	volatile uint32_t CR2;						/* I2C control register 2						Offset: 0x04 */
	volatile uint32_t OAR1;						/* I2C own address register	1					Offset: 0x08 */
	volatile uint32_t OAR2;						/* I2C own address register	2				 	Offset: 0x0C */
	volatile uint32_t DR;						/* I2C data register							Offset: 0x10 */
	volatile uint32_t SR1;						/* I2C status register 1						Offset: 0x14 */
	volatile uint32_t SR2;						/* I2C status register 2						Offset: 0x18 */
	volatile uint32_t CCR;						/* I2C clock control register					Offset: 0x1C */
	volatile uint32_t TRISE;					/* I2C TRISE register						 	Offset: 0x20 */
	volatile uint32_t FLTR;						/* I2C FLTR register						 	Offset: 0x20 */
} I2C_RegDef_t;


/* Reset and Clock Control (RCC) */

typedef struct
{
	volatile uint32_t CR;						/* RCC clock control register							Offset: 0x00 */
	volatile uint32_t PLLCFGR;					/* RCC PLL configuration register						Offset: 0x04 */
	volatile uint32_t CFGR;						/* RCC clock configuration register						Offset: 0x08 */
	volatile uint32_t CIR;						/* RCC clock interrupt register							Offset: 0x0C */
	volatile uint32_t AHB1RSTR;					/* RCC AHB1 peripheral reset register					Offset: 0x10 */
	volatile uint32_t AHB2RSTR;					/* RCC AHB2 peripheral reset register					Offset: 0x14 */
	volatile uint32_t AHB3RSTR;					/* RCC AHB3 peripheral reset register					Offset: 0x18 */
	volatile uint32_t RESERVED0;				/*														Offset: 0x1C */
	volatile uint32_t APB1RSTR;					/* RCC APB1 peripheral reset register					Offset: 0x20 */
	volatile uint32_t APB2RSTR;					/* RCC APB2 peripheral reset register					Offset: 0x24 */
	volatile uint32_t RESERVED1;				/*														Offset: 0x28 */
	volatile uint32_t RESERVED2;				/*														Offset: 0x2C */
	volatile uint32_t AHB1ENR;					/* RCC AHB1 peripheral clock enable register			Offset: 0x30 */
	volatile uint32_t AHB2ENR;					/* RCC AHB2 peripheral clock enable register			Offset: 0x34 */
	volatile uint32_t AHB3ENR;					/* RCC AHB3 peripheral clock enable register			Offset: 0x38 */
	volatile uint32_t RESERVED3;				/*														Offset: 0x3C */
	volatile uint32_t APB1ENR;					/* RCC APB1 peripheral clock enable register			Offset: 0x40 */
	volatile uint32_t APB2ENR;					/* RCC APB2 peripheral clock enable register			Offset: 0x44 */
	volatile uint32_t RESERVED4;				/*														Offset: 0x48 */
	volatile uint32_t RESERVED5;				/*														Offset: 0x4C */
	volatile uint32_t AHB1LPENR;				/* RCC AHB1 peripheral clock enable low power register	Offset: 0x50 */
	volatile uint32_t AHB2LPENR;				/* RCC AHB2 peripheral clock enable low power register	Offset: 0x54 */
	volatile uint32_t AHB3LPENR;				/* RCC AHB3 peripheral clock enable low power register	Offset: 0x58 */
	volatile uint32_t RESERVED6;				/*														Offset: 0x5C */
	volatile uint32_t APB1LPENR;				/* RCC APB1 peripheral clock enable low power register	Offset: 0x60 */
	volatile uint32_t APB2LPENR;				/* RCC APB2 peripheral clock enable low power register	Offset: 0x64 */
	volatile uint32_t RESERVED7;				/*														Offset: 0x68 */
	volatile uint32_t RESERVED8;				/*														Offset: 0x6C */
	volatile uint32_t BDCR;						/* RCC backup domain control register					Offset: 0x70 */
	volatile uint32_t CSR;						/* RCC clock control and status register				Offset: 0x74 */
	volatile uint32_t RESERVED9;				/*														Offset: 0x78 */
	volatile uint32_t RESERVED10;				/*														Offset: 0x7C */
	volatile uint32_t SSCGR;					/* RCC spread spectrum clock generation register		Offset: 0x80 */
	volatile uint32_t PLLI2SCFGR;				/* RCC PLLI2S configuration register					Offset: 0x84 */
	volatile uint32_t PLLSAICFGR;				/* RCC PLL configuration register						Offset: 0x88 */
	volatile uint32_t DCKCFGR;					/* RCC dedicated clock configuration register			Offset: 0x8C */
}RCC_RegDef_t;


/* External Interrupt (EXTI) */

typedef struct
{
	volatile uint32_t IMR;						/* 														Offset: 0x00 */
	volatile uint32_t EMR;						/* 														Offset: 0x04 */
	volatile uint32_t RTSR;						/*														Offset: 0x08 */
	volatile uint32_t FTSR;						/* 														Offset: 0x0C */
	volatile uint32_t SWIER;					/* 														Offset: 0x10 */
	volatile uint32_t PR;						/* 														Offset: 0x14 */
}EXTI_RegDef_t;


/* System Configuration (SYSCFG) */

typedef struct
{
	volatile uint32_t MEMRMP;					/* SYSCFG memory remap register							Offset: 0x00 */
	volatile uint32_t PMC;						/* Peripheral mode configuration register				Offset: 0x04 */
	volatile uint32_t EXTICR[4];				/* EXTI configuration register 1-4						Offset: 0x08-0x14 */
	volatile uint32_t RESERVED[2];				/* 														Offset: 0x18-0x1C */
	volatile uint32_t CMPCR;					/* Compensation cell control register					Offset: 0x20 */
}SYSCFG_RegDef_t;


/********************************************************************************/

/* Peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t)  */

/********************************************************************************/


/* GPIOs */

#define GPIOA				((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI				((GPIO_RegDef_t*) GPIOI_BASEADDR)


/* SPI */

#define SPI1				((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*) SPI3_BASEADDR)


/* I2C */

#define I2C1				((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3				((I2C_RegDef_t*) I2C3_BASEADDR)


/* Reset and Clock Control (RCC) */

#define RCC					((RCC_RegDef_t*) RCC_BASEADDR)


/* External Interrupt (EXTI) */

#define EXTI				((EXTI_RegDef_t*) EXTI_BASEADDR)


/* System Configuration (SYSCFG) */

#define SYSCFG				((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)


/********************************************************************************/

/* Clock enable macros for peripherals */

/********************************************************************************/


/* GPIOs */

#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()		( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()		( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()		( RCC->AHB1ENR |= (1 << 8) )


/* SPI */

#define SPI1_PCLK_EN()		( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()		( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= (1 << 15) )


/* I2C */

#define I2C1_PCLK_EN()		( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= (1 << 23) )


/* SYSCFG */

#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= (1 << 14) )


/********************************************************************************/

/* Clock disable macros for peripherals */

/********************************************************************************/


/* GPIOs */

#define GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 8) )


/* SPI */

#define SPI1_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 15) )


/* I2C */

#define I2C1_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 23) )


/* SYSCFG */

#define SYSCFG_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 14) )


/********************************************************************************/

/* Macros to reset GPIOx peripherals */

/********************************************************************************/

#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 0)); RCC->AHB1RSTR &= ~(1 << 0);} while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 1)); RCC->AHB1RSTR &= ~(1 << 1);} while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 2)); RCC->AHB1RSTR &= ~(1 << 2);} while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 3)); RCC->AHB1RSTR &= ~(1 << 3);} while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 4)); RCC->AHB1RSTR &= ~(1 << 4);} while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 5)); RCC->AHB1RSTR &= ~(1 << 5);} while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 6)); RCC->AHB1RSTR &= ~(1 << 6);} while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 7)); RCC->AHB1RSTR &= ~(1 << 7);} while(0)
#define GPIOI_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 8)); RCC->AHB1RSTR &= ~(1 << 8);} while(0)


/********************************************************************************/

/* Macro to get port code from GPIO base address */

/********************************************************************************/

#define GPIO_BASEADDR_TO_CODE(x)	( (x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
									(x == GPIOH) ? 7 :\
									(x == GPIOI) ? 8 :0 )


/********************************************************************************/

/* EXTI line IRQ (Interrupt request) numbers */

/********************************************************************************/

#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40


/* Macros for all possible IRQ priority positions */

#define NVIC_IRQ_PRI0			0
#define NVIC_IRQ_PRI1			1
#define NVIC_IRQ_PRI2			2
#define NVIC_IRQ_PRI3			3
#define NVIC_IRQ_PRI4			4
#define NVIC_IRQ_PRI5			5
#define NVIC_IRQ_PRI6			6
#define NVIC_IRQ_PRI7			7
#define NVIC_IRQ_PRI8			8
#define NVIC_IRQ_PRI9			9
#define NVIC_IRQ_PRI10			10
#define NVIC_IRQ_PRI11			11
#define NVIC_IRQ_PRI12			12
#define NVIC_IRQ_PRI13			13
#define NVIC_IRQ_PRI14			14
#define NVIC_IRQ_PRI15			15


/********************************************************************************/

/* Bit position definitions of SPI peripheral */

/********************************************************************************/

/* CR1 register */

#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_BIDIMODE		15


/* CR2 register */

#define SPI_CR2_CRXDMAEN		0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7


/* SR register */

#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8


/********************************************************************************/

/* SPI IRQ (Interrupt request) numbers */

/********************************************************************************/

#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51


/********************************************************************************/

/* Macros to reset SPIx peripherals */

/********************************************************************************/

#define SPI1_REG_RESET()			do{ (RCC->APB2RSTR |= (1 << 12)); RCC->APB2RSTR &= ~(1 << 12);} while(0)
#define SPI2_REG_RESET()			do{ (RCC->APB1RSTR |= (1 << 14)); RCC->APB1RSTR &= ~(1 << 14);} while(0)
#define SPI3_REG_RESET()			do{ (RCC->APB1RSTR |= (1 << 15)); RCC->APB1RSTR &= ~(1 << 15);} while(0)


/********************************************************************************/

/* Bit position definitions of I2C peripheral */

/********************************************************************************/

/* CR1 register */

#define I2C_CR1_PE				0
#define I2C_CR1_SMBUS			1
#define I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15


/* CR2 register */

#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12


/* Status register 1 */

#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RXNE			6
#define I2C_SR1_TXE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_PECERR			12
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15


/* Status register 2 */

#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_SMBDEAFULT		5
#define I2C_SR2_SBHOST			6
#define I2C_SR2_DUALF			7
#define I2C_SR2_PEC				8


/* Clock control register */

#define I2C_CCR_CCR				0
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS				15


/********************************************************************************/

/* Macros to reset I2Cx peripherals */

/********************************************************************************/

#define I2C1_REG_RESET()			do{ (RCC->APB1RSTR |= (1 << 21)); RCC->APB1RSTR &= ~(1 << 21);} while(0)
#define I2C2_REG_RESET()			do{ (RCC->APB1RSTR |= (1 << 22)); RCC->APB1RSTR &= ~(1 << 22);} while(0)
#define I2C3_REG_RESET()			do{ (RCC->APB1RSTR |= (1 << 23)); RCC->APB1RSTR &= ~(1 << 23);} while(0)


/********************************************************************************/

/* I2C IRQ (Interrupt request) numbers */

/********************************************************************************/

#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32
#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34
#define IRQ_NO_I2C3_EV			72
#define IRQ_NO_I2C3_ER			73


/********************************************************************************/

/* Other macros */

/********************************************************************************/

#define ENABLE				1
#define DISABLE 			0
#define SET					ENABLE
#define RESET				DISABLE
#define FLAG_SET			SET
#define FLAG_RESET			RESET
#define LOW					0
#define HIGH				1
#define WRITE				0
#define READ				1


/********************************************************************************/

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
//#include "ssd1306_driver.h"
//#include "max30102_driver.h"


#endif /* INC_STM32F407XX_H_ */

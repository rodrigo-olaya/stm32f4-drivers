/*
 * stm32f407.h
 *
 *  Created on: Apr 23, 2025
 *      Author: rodrigoolaya
 */

#ifndef INC_STM32F407_H_
#define INC_STM32F407_H_

#include <stdint.h>

/*
 * Base address of flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U	// 112 KB
#define SRAM2_BASEADDR			0x2001C000U	// 16KB, CALCULATED using SRAM1base + 112KB
#define ROM_BASEADDR			0x1FFF0000U // SYSTEM MEMORY
#define SRAM 					SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base address
 */

#define PERIPH_BASEADDR			0X40000000U
#define	APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0X40010000U
#define	AHB1PERIPH_BASEADDR		0X40020000U
#define AHB2PERIPH_BASEADDR		0X50000000U

/*
 * Base address of all peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0X0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0X0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0X0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0X0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0X1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0X1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0X1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0X1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR + 0X2000)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)


/*
 * Base addresses of peripherals for APB1
 */

#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)


/*
 * Base addresses of peripherals for APB2
 */

#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)

/*
 * Peripheral Register Definition Structures
 */

typedef struct
{
	volatile uint32_t MODER;		// Address offset: 0x00
	volatile uint32_t OTYPER;		// Address offset: 0x04
	volatile uint32_t OSPEEDR;		// Address offset: 0x08
	volatile uint32_t PUPDR;		// Address offset: 0x0C
	volatile uint32_t IDR;			// Address offset: 0x10
	volatile uint32_t ODR;			// Address offset: 0x14
	volatile uint32_t BSRR;			// Address offset: 0x18
	volatile uint32_t LCKR;			// Address offset: 0x1C
	volatile uint32_t AFR[2];		// Address offset: 0x20
}GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
	volatile uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
	volatile uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
	volatile uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
	volatile uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
	volatile uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
	volatile uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
	uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
	volatile uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
	volatile uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
	uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
	volatile uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
	volatile uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
	volatile uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
	uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
	volatile uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
	volatile uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
	uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
	volatile uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
	volatile uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
	volatile uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
	uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
	volatile uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
	volatile uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
	uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
	volatile uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
	volatile uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
	uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
	volatile uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
	volatile uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
	volatile uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
	volatile uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
	volatile uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
	volatile uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */

} RCC_RegDef_t;

typedef struct
{
	volatile uint32_t CR1;	/* Address offset 0x00 */
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;

} SPI_RegDef_t;

/*
 * Peripheral definitions (Type casting)
 */

#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI		((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()	(RCC->AHB1ENR |= (1 << 8))

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PLCK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))

/*
 * Clock Enable Macros for SPI peripherals
 */

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCCK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCCK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCCK_EN()  (RCC->APB1ENR |= (1 << 19))
#define UART5_PCCK_EN()  (RCC->APB1ENR |= (1 << 20))
#define USART6_PCCK_EN() (RCC->APB1ENR |= (1 << 5))

/*
 * Clock enable Macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

/*
 * Clock disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock disable Macros for SPI
 */

#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_Di() (RCC->APB2ENR &= ~(1 << 13))


/*
 * Macros to reset RCC for GPIO
 */

#define GPIOA_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOF_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define GPIOG_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));} while(0)
#define GPIOI_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));} while(0)

/*
 * Macros to reset RCC for SPI
 */

#define SPI1_REG_RESET()		do{ (RCC->APB2ENR |= (1 << 12)); (RCC->APB2ENR &= ~(1 << 12));} while(0)
#define SPI2_REG_RESET()		do{ (RCC->APB1ENR |= (1 << 14)); (RCC->APB1ENR &= ~(1 << 14));} while(0)
#define SPI3_REG_RESET()		do{ (RCC->APB1ENR |= (1 << 15)); (RCC->APB1ENR &= ~(1 << 15));} while(0)
#define SPI4_REG_RESET()		do{ (RCC->APB2ENR |= (1 << 13)); (RCC->APB2ENR &= ~(1 << 13));} while(0)

/*
 * some generic Macros for enabling/disabling
 */

#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET  RESET
#define FLAG_SET		SET
#define FLAG_RESET 		RESET

/*
 * Macros for the Bits in the SPI Control Register 1, as seen in 28.5.1 of RM
 */

#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RX_ONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDI_MODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7

/*
 * Macros for the bits in SPI Status Register, as seen in 28.5.3 of RM
 */

#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8


#endif /* INC_STM32F407_H_ */

/*
 * Copyright (c) 2026 BIII TECH LLP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Register layout and bit names for the CH5xx I2C controller.
 *
 * The block is the same one used on the ch32v parts, but the WCH HAL
 * describes it only through absolute-address macros and the RB_I2C_* bit
 * names, so neither I2C_TypeDef nor the I2C_* names the driver expects are
 * available. Both are provided here in terms of the HAL definitions, so the
 * bit values keep a single source of truth.
 */

#ifndef ZEPHYR_DRIVERS_I2C_I2C_WCH_CH5XX_H_
#define ZEPHYR_DRIVERS_I2C_I2C_WCH_CH5XX_H_

#include <stdint.h>

#include <hal_ch32fun.h>

typedef struct {
	volatile uint16_t CTLR1; /* 0x00 */
	volatile uint16_t RESERVED0;
	volatile uint16_t CTLR2; /* 0x04 */
	volatile uint16_t RESERVED1;
	volatile uint16_t OADDR1; /* 0x08 */
	volatile uint16_t RESERVED2;
	volatile uint16_t OADDR2; /* 0x0c */
	volatile uint16_t RESERVED3;
	volatile uint16_t DATAR; /* 0x10 */
	volatile uint16_t RESERVED4;
	volatile uint16_t STAR1; /* 0x14 */
	volatile uint16_t RESERVED5;
	volatile uint16_t STAR2; /* 0x18 */
	volatile uint16_t RESERVED6;
	volatile uint16_t CKCFGR; /* 0x1c */
	volatile uint16_t RESERVED7;
	volatile uint16_t RTR; /* 0x20 */
} I2C_TypeDef;

#define I2C_CTLR1_PE    RB_I2C_PE
#define I2C_CTLR1_START RB_I2C_START
#define I2C_CTLR1_STOP  RB_I2C_STOP
#define I2C_CTLR1_ACK   RB_I2C_ACK
#define I2C_CTLR1_POS   RB_I2C_POS

#define I2C_CTLR2_FREQ    RB_I2C_FREQ
#define I2C_CTLR2_ITERREN RB_I2C_ITERREN
#define I2C_CTLR2_ITEVTEN RB_I2C_ITEVTEN
#define I2C_CTLR2_ITBUFEN RB_I2C_ITBUFEN

#define I2C_STAR1_SB   RB_I2C_SB
#define I2C_STAR1_ADDR RB_I2C_ADDR
#define I2C_STAR1_BTF  RB_I2C_BTF
#define I2C_STAR1_RXNE RB_I2C_RxNE
#define I2C_STAR1_TXE  RB_I2C_TxE
#define I2C_STAR1_BERR RB_I2C_BERR
#define I2C_STAR1_ARLO RB_I2C_ARLO
#define I2C_STAR1_AF   RB_I2C_AF

#define I2C_STAR2_BUSY RB_I2C_BUSY

#define I2C_CKCFGR_FS RB_I2C_F_S

#define I2C_RTR_TRISE RB_I2C_TRISE

#endif /* ZEPHYR_DRIVERS_I2C_I2C_WCH_CH5XX_H_ */

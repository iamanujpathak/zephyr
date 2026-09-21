/*
 * Copyright (c) 2025 BIII TECH LLP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Register layout for the CH5xx SPI controller.
 *
 * The WCH HAL exposes this block only through absolute-address macros, which
 * cannot be used with the base address taken from devicetree. The bit and mask
 * definitions are reused from the HAL; only the register layout and the
 * accessors are defined here.
 */

#ifndef ZEPHYR_DRIVERS_SPI_SPI_WCH_CH5XX_H_
#define ZEPHYR_DRIVERS_SPI_SPI_WCH_CH5XX_H_

#include <stdbool.h>
#include <stdint.h>

#include <hal_ch32fun.h>

typedef struct {
	volatile uint8_t CTRL_MOD; /* 0x00 */
	volatile uint8_t CTRL_CFG; /* 0x01 */
	volatile uint8_t INTER_EN; /* 0x02 */
	union {
		volatile uint8_t CLOCK_DIV; /* 0x03, controller mode */
		volatile uint8_t SLAVE_PRE; /* 0x03, peripheral mode */
	};
	volatile uint8_t BUFFER;     /* 0x04 */
	volatile uint8_t RUN_FLAG;   /* 0x05 */
	volatile uint8_t INT_FLAG;   /* 0x06 */
	volatile uint8_t FIFO_COUNT; /* 0x07 */
	volatile uint8_t INT_TYPE;   /* 0x08 */
	volatile uint8_t INTER1_EN;  /* 0x09 */
	volatile uint8_t INT1_FLAG;  /* 0x0a */
	volatile uint8_t RESERVED0;
	volatile uint16_t TOTAL_CNT; /* 0x0c */
	volatile uint16_t RESERVED1;
	volatile uint8_t FIFO; /* 0x10 */
	volatile uint8_t RESERVED2;
	volatile uint8_t RESERVED3;
	volatile uint8_t FIFO_COUNT1; /* 0x13 */
	volatile uint32_t DMA_NOW;    /* 0x14 */
	volatile uint32_t DMA_BEG;    /* 0x18 */
	volatile uint32_t DMA_END;    /* 0x1c */
} SPI_TypeDef;

#define CH5XX_SPI_CLOCK_DIV_MIN 2
#define CH5XX_SPI_CLOCK_DIV_MAX 254

static inline void ch5xx_spi_write_bit(volatile uint8_t *reg, uint8_t mask, bool set)
{
	if (set) {
		*reg |= mask;
	} else {
		*reg &= ~mask;
	}
}

static inline void ch5xx_spi_set_clock_frequency(SPI_TypeDef *regs, uint32_t fsys,
						 uint32_t frequency)
{
	uint32_t div = fsys / frequency;

	if (div < CH5XX_SPI_CLOCK_DIV_MIN) {
		div = CH5XX_SPI_CLOCK_DIV_MIN;
	} else if (div > CH5XX_SPI_CLOCK_DIV_MAX) {
		div = CH5XX_SPI_CLOCK_DIV_MAX;
	}

	regs->CLOCK_DIV = (uint8_t)div;
}

static inline void ch5xx_spi_set_master_mode(SPI_TypeDef *regs, bool master)
{
	ch5xx_spi_write_bit(&regs->CTRL_MOD, RB_SPI_MODE_SLAVE, !master);
	ch5xx_spi_write_bit(&regs->CTRL_CFG, RB_SPI_AUTO_IF, true);
}

static inline void ch5xx_spi_set_mode(SPI_TypeDef *regs, bool cpol, bool cpha)
{
	ch5xx_spi_write_bit(&regs->CTRL_MOD, RB_SPI_MST_SCK_MOD, cpol);
	/* The clock select bit has the opposite sense to CPHA */
	ch5xx_spi_write_bit(&regs->CTRL_CFG, RB_MST_CLK_SEL, cpha);
}

static inline void ch5xx_spi_set_fifo_dir(SPI_TypeDef *regs, bool rx)
{
	ch5xx_spi_write_bit(&regs->CTRL_MOD, RB_SPI_FIFO_DIR, rx);
}

static inline void ch5xx_spi_set_pin_mode(SPI_TypeDef *regs, bool mosi, bool miso, bool sck)
{
	ch5xx_spi_write_bit(&regs->CTRL_MOD, RB_SPI_MOSI_OE, mosi);
	ch5xx_spi_write_bit(&regs->CTRL_MOD, RB_SPI_MISO_OE, miso);
	ch5xx_spi_write_bit(&regs->CTRL_MOD, RB_SPI_SCK_OE, sck);
}

static inline void ch5xx_spi_cfg_dma(SPI_TypeDef *regs, uint8_t *buf, uint32_t len, bool rx)
{
	ch5xx_spi_set_fifo_dir(regs, rx);
	regs->DMA_BEG = (uint32_t)(uintptr_t)buf;
	regs->DMA_END = (uint32_t)(uintptr_t)(buf + len);
	regs->TOTAL_CNT = len;
}

static inline void ch5xx_spi_start_dma(SPI_TypeDef *regs, bool interrupt)
{
	ch5xx_spi_write_bit(&regs->INTER_EN, RB_SPI_IE_DMA_END, interrupt);
	ch5xx_spi_write_bit(&regs->INTER_EN, RB_SPI_IE_CNT_END, interrupt);
	ch5xx_spi_write_bit(&regs->CTRL_CFG, RB_SPI_DMA_ENABLE, true);
}

static inline void ch5xx_spi_stop_dma(SPI_TypeDef *regs)
{
	ch5xx_spi_write_bit(&regs->INTER_EN, RB_SPI_IE_DMA_END, false);
	ch5xx_spi_write_bit(&regs->INTER_EN, RB_SPI_IE_CNT_END, false);
	ch5xx_spi_write_bit(&regs->CTRL_CFG, RB_SPI_DMA_ENABLE, false);
}

static inline uint8_t ch5xx_spi_get_int_flag(SPI_TypeDef *regs)
{
	return regs->INT_FLAG;
}

static inline bool ch5xx_spi_int_flag_has_dma_end(uint8_t flags)
{
	return (flags & RB_SPI_IF_DMA_END) != 0;
}

static inline bool ch5xx_spi_int_flag_has_cnt_end(uint8_t flags)
{
	return (flags & RB_SPI_IF_CNT_END) != 0;
}

static inline void ch5xx_spi_clear_all(SPI_TypeDef *regs)
{
	/* Clears the FIFO, the transfer counter and the interrupt flags */
	ch5xx_spi_write_bit(&regs->CTRL_MOD, RB_SPI_ALL_CLEAR, true);
}

#endif /* ZEPHYR_DRIVERS_SPI_SPI_WCH_CH5XX_H_ */

/*
 * Copyright (c) 2026 BIII TECH LLP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Register layout for the CH5xx USB full-speed device controller.
 *
 * The WCH HAL only exposes this block through absolute-address macros, which
 * cannot be used with the base address taken from devicetree. The bit and mask
 * definitions are reused from the HAL; only the register layout and the
 * indexed accessors are defined here.
 */

#ifndef ZEPHYR_DRIVERS_USB_UDC_UDC_WCH_CH5XX_H_
#define ZEPHYR_DRIVERS_USB_UDC_UDC_WCH_CH5XX_H_

#include <stdbool.h>
#include <stdint.h>

#include <hal_ch32fun.h>

#define CH5XX_USB_EP_MPS  64
#define CH5XX_USB_NUM_EPS 8

/* Endpoint control block: T_LEN at +0, CTRL at +2 */
typedef struct {
	volatile uint8_t T_LEN;
	volatile uint8_t RESERVED;
	volatile uint8_t CTRL;
	volatile uint8_t RESERVED1;
} USB_EP_CTRL_TypeDef;

typedef struct {
	volatile uint8_t CTRL;      /* 0x00 */
	volatile uint8_t UDEV_CTRL; /* 0x01 */
	volatile uint8_t INT_EN;    /* 0x02 */
	volatile uint8_t DEV_AD;    /* 0x03 */
	volatile uint8_t RESERVED0; /* 0x04 */
	volatile uint8_t MIS_ST;    /* 0x05 */
	volatile uint8_t INT_FG;    /* 0x06 */
	volatile uint8_t INT_ST;    /* 0x07 */
	volatile uint8_t RX_LEN;    /* 0x08 */
	volatile uint8_t RESERVED1[3];
	volatile uint8_t UEP4_1_MOD; /* 0x0c */
	volatile uint8_t UEP2_3_MOD; /* 0x0d */
	volatile uint8_t UEP567_MOD; /* 0x0e */
	volatile uint8_t RESERVED2;
	volatile uint16_t UEP0_DMA; /* 0x10 */
	volatile uint16_t RESERVED3;
	volatile uint16_t UEP1_DMA; /* 0x14 */
	volatile uint16_t RESERVED4;
	volatile uint16_t UEP2_DMA; /* 0x18 */
	volatile uint16_t RESERVED5;
	volatile uint16_t UEP3_DMA; /* 0x1c */
	volatile uint16_t RESERVED6;
	USB_EP_CTRL_TypeDef EP[5]; /* 0x20: EP0..EP4 */
} USB_TypeDef;

/*
 * Endpoints 5 to 7 sit outside the struct: their DMA registers start at
 * offset 0x54 and their control registers at 0x64, both with a 4 byte stride.
 */
#define CH5XX_USB_EP567_DMA_OFFSET  0x54
#define CH5XX_USB_EP567_TLEN_OFFSET 0x64
#define CH5XX_USB_EP567_CTRL_OFFSET 0x66

static inline void ch5xx_usb_write_bit(volatile uint8_t *reg, uint8_t mask, bool set)
{
	if (set) {
		*reg |= mask;
	} else {
		*reg &= ~mask;
	}
}

static inline volatile uint8_t *ch5xx_usb_ep_ctrl_ptr(USB_TypeDef *usb, uint8_t ep)
{
	if (ep <= 4) {
		return &usb->EP[ep].CTRL;
	}

	return (volatile uint8_t *)((uintptr_t)usb + CH5XX_USB_EP567_CTRL_OFFSET + (ep - 5) * 4);
}

static inline volatile uint8_t *ch5xx_usb_ep_tlen_ptr(USB_TypeDef *usb, uint8_t ep)
{
	if (ep <= 4) {
		return &usb->EP[ep].T_LEN;
	}

	return (volatile uint8_t *)((uintptr_t)usb + CH5XX_USB_EP567_TLEN_OFFSET + (ep - 5) * 4);
}

static inline volatile uint16_t *ch5xx_usb_ep_dma_ptr(USB_TypeDef *usb, uint8_t ep)
{
	/* EP0 to EP3 are contiguous from 0x10 with a 4 byte stride */
	if (ep <= 3) {
		return &usb->UEP0_DMA + (ep * 2);
	}

	/* EP4 shares the EP0 buffer and has no DMA register of its own */
	if (ep == 4) {
		return &usb->UEP0_DMA;
	}

	return (volatile uint16_t *)((uintptr_t)usb + CH5XX_USB_EP567_DMA_OFFSET + (ep - 5) * 4);
}

static inline void ch5xx_usb_ep_set_rx_res(USB_TypeDef *usb, uint8_t ep, uint8_t res)
{
	volatile uint8_t *ctrl = ch5xx_usb_ep_ctrl_ptr(usb, ep);

	*ctrl = (*ctrl & ~MASK_UEP_R_RES) | (res & MASK_UEP_R_RES);
}

static inline void ch5xx_usb_ep_set_tx_res(USB_TypeDef *usb, uint8_t ep, uint8_t res)
{
	volatile uint8_t *ctrl = ch5xx_usb_ep_ctrl_ptr(usb, ep);

	*ctrl = (*ctrl & ~MASK_UEP_T_RES) | (res & MASK_UEP_T_RES);
}

static inline void ch5xx_usb_ep_set_tx_len(USB_TypeDef *usb, uint8_t ep, uint8_t len)
{
	*ch5xx_usb_ep_tlen_ptr(usb, ep) = len;
}

static inline void ch5xx_usb_ep_set_tx_tog(USB_TypeDef *usb, uint8_t ep, bool data1)
{
	ch5xx_usb_write_bit(ch5xx_usb_ep_ctrl_ptr(usb, ep), RB_UEP_T_TOG, data1);
}

static inline void ch5xx_usb_ep_set_rx_tog(USB_TypeDef *usb, uint8_t ep, bool data1)
{
	ch5xx_usb_write_bit(ch5xx_usb_ep_ctrl_ptr(usb, ep), RB_UEP_R_TOG, data1);
}

static inline void ch5xx_usb_ep_set_auto_tog(USB_TypeDef *usb, uint8_t ep, bool enable)
{
	ch5xx_usb_write_bit(ch5xx_usb_ep_ctrl_ptr(usb, ep), RB_UEP_AUTO_TOG, enable);
}

static inline void ch5xx_usb_ep_set_tx_en(USB_TypeDef *usb, uint8_t ep, bool enable)
{
	switch (ep) {
	case 1:
		ch5xx_usb_write_bit(&usb->UEP4_1_MOD, RB_UEP1_TX_EN, enable);
		break;
	case 2:
		ch5xx_usb_write_bit(&usb->UEP2_3_MOD, RB_UEP2_TX_EN, enable);
		break;
	case 3:
		ch5xx_usb_write_bit(&usb->UEP2_3_MOD, RB_UEP3_TX_EN, enable);
		break;
	case 4:
		ch5xx_usb_write_bit(&usb->UEP4_1_MOD, RB_UEP4_TX_EN, enable);
		break;
	case 5:
		ch5xx_usb_write_bit(&usb->UEP567_MOD, RB_UEP5_TX_EN, enable);
		break;
	case 6:
		ch5xx_usb_write_bit(&usb->UEP567_MOD, RB_UEP6_TX_EN, enable);
		break;
	case 7:
		ch5xx_usb_write_bit(&usb->UEP567_MOD, RB_UEP7_TX_EN, enable);
		break;
	default:
		break;
	}
}

static inline void ch5xx_usb_ep_set_rx_en(USB_TypeDef *usb, uint8_t ep, bool enable)
{
	switch (ep) {
	case 1:
		ch5xx_usb_write_bit(&usb->UEP4_1_MOD, RB_UEP1_RX_EN, enable);
		break;
	case 2:
		ch5xx_usb_write_bit(&usb->UEP2_3_MOD, RB_UEP2_RX_EN, enable);
		break;
	case 3:
		ch5xx_usb_write_bit(&usb->UEP2_3_MOD, RB_UEP3_RX_EN, enable);
		break;
	case 4:
		ch5xx_usb_write_bit(&usb->UEP4_1_MOD, RB_UEP4_RX_EN, enable);
		break;
	case 5:
		ch5xx_usb_write_bit(&usb->UEP567_MOD, RB_UEP5_RX_EN, enable);
		break;
	case 6:
		ch5xx_usb_write_bit(&usb->UEP567_MOD, RB_UEP6_RX_EN, enable);
		break;
	case 7:
		ch5xx_usb_write_bit(&usb->UEP567_MOD, RB_UEP7_RX_EN, enable);
		break;
	default:
		break;
	}
}

static inline void ch5xx_usb_ep_set_dma(USB_TypeDef *usb, uint8_t ep, void *buf)
{
	/* The controller only latches the lower 16 bits of the SRAM address */
	*ch5xx_usb_ep_dma_ptr(usb, ep) = (uint16_t)((uintptr_t)buf);
}

static inline uint8_t *ch5xx_usb_ep_get_dma_buf(USB_TypeDef *usb, uint8_t ep)
{
	uint16_t dma = *ch5xx_usb_ep_dma_ptr(usb, ep);

	return (uint8_t *)(CONFIG_SRAM_BASE_ADDRESS + dma);
}

static inline uint8_t ch5xx_usb_get_int_fg(USB_TypeDef *usb)
{
	return usb->INT_FG;
}

static inline void ch5xx_usb_clear_int_fg(USB_TypeDef *usb, uint8_t flags)
{
	usb->INT_FG = flags;
}

static inline uint8_t ch5xx_usb_get_int_st(USB_TypeDef *usb)
{
	return usb->INT_ST;
}

static inline uint8_t ch5xx_usb_get_rx_len(USB_TypeDef *usb)
{
	return usb->RX_LEN;
}

static inline uint8_t ch5xx_usb_int_st_ep(uint8_t int_st)
{
	return int_st & MASK_UIS_ENDP;
}

static inline uint8_t ch5xx_usb_int_st_token(uint8_t int_st)
{
	return int_st & MASK_UIS_TOKEN;
}

static inline bool ch5xx_usb_int_st_is_setup(uint8_t int_st)
{
	return (int_st & RB_UIS_SETUP_ACT) != 0;
}

static inline bool ch5xx_usb_tog_ok(USB_TypeDef *usb)
{
	return (usb->INT_FG & RB_U_TOG_OK) != 0;
}

static inline void ch5xx_usb_set_addr(USB_TypeDef *usb, uint8_t addr)
{
	usb->DEV_AD = (usb->DEV_AD & ~MASK_USB_ADDR) | (addr & MASK_USB_ADDR);
}

static inline void ch5xx_usb_reset_sie(USB_TypeDef *usb)
{
	usb->CTRL |= RB_UC_RESET_SIE;
	usb->CTRL &= ~RB_UC_RESET_SIE;
}

static inline void ch5xx_usb_clr_all(USB_TypeDef *usb)
{
	usb->CTRL |= RB_UC_CLR_ALL;
	usb->CTRL &= ~RB_UC_CLR_ALL;
}

static inline void ch5xx_usb_dev_enable(USB_TypeDef *usb, bool pullup)
{
	usb->CTRL = RB_UC_DMA_EN | RB_UC_INT_BUSY |
		    (pullup ? RB_UC_DEV_PU_EN | RB_UC_SYS_CTRL0 : 0);
	usb->UDEV_CTRL = RB_UD_PD_DIS | RB_UD_PORT_EN;
}

static inline void ch5xx_usb_dev_pullup(USB_TypeDef *usb, bool enable)
{
	ch5xx_usb_write_bit(&usb->CTRL, RB_UC_DEV_PU_EN | RB_UC_SYS_CTRL0, enable);
}

static inline void ch5xx_usb_set_int_en(USB_TypeDef *usb, uint8_t mask)
{
	usb->INT_EN = mask;
}

#endif /* ZEPHYR_DRIVERS_USB_UDC_UDC_WCH_CH5XX_H_ */

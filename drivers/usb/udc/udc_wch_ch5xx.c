/*
 * Copyright (c) 2026 BIII Tech LLP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB Device Controller driver for WCH CH5xx (CH572/CH573)
 *
 * This driver implements the Zephyr UDC (device_next) API for the WCH CH572
 * full-speed USB device controller. The hardware provides 8 bidirectional
 * endpoints (EP0-EP7), each with 64-byte maximum packet size.
 *
 * Architecture:
 * - ISR reads hardware state and posts events to a dedicated thread
 * - Thread processes SETUP packets, starts new transfers, and handles
 *   deferred OUT data when no buffer was available in ISR context
 * - EP1-3 support auto-toggle; EP0,4-7 require manual toggle management
 * - EP4 shares DMA base with EP0 (not used in this driver)
 *
 * Buffer layout per endpoint:
 * - EP0: 64-byte shared TX/RX buffer at DMA address
 * - EP1-3,5-7: 128-byte buffer [64B RX][64B TX], DMA points to start
 */

#define DT_DRV_COMPAT wch_ch5xx_usb

#include "udc_common.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/sys/byteorder.h>

#include "udc_wch_ch5xx.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udc_wch_ch5xx, CONFIG_UDC_DRIVER_LOG_LEVEL);

/* Number of hardware endpoints */
#define NUM_OF_HW_EPS		8
#define EP0_MPS			64
#define EP_BUF_SIZE		64

/* Thread event bits */
#define EVT_SETUP		BIT(0)
#define EVT_XFER_NEXT		BIT(1)
#define EVT_OUT_PENDING		BIT(2)
#define EVT_ALL			(EVT_SETUP | EVT_XFER_NEXT | EVT_OUT_PENDING)

/**
 * Check if endpoint has auto-toggle capability.
 * Only EP1-3 support RB_UEP_AUTO_TOG.
 */
static inline bool ep_has_auto_tog(uint8_t ep_hw)
{
	return ep_hw >= 1 && ep_hw <= 3;
}

/**
 * Get pointer to the RX buffer region for an endpoint.
 * EP0: DMA base (shared TX/RX)
 * EP1-3,5-7: DMA base + 0 (first 64 bytes)
 */
static inline uint8_t *ep_rx_buf(USB_TypeDef *usb, uint8_t ep_hw)
{
	return ch5xx_usb_ep_get_dma_buf(usb, ep_hw);
}

/**
 * Get pointer to the TX buffer region for an endpoint.
 * EP0: DMA base (shared TX/RX)
 * EP1-3,5-7: DMA base + 64 (second 64 bytes)
 */
static inline uint8_t *ep_tx_buf(USB_TypeDef *usb, uint8_t ep_hw)
{
	if (ep_hw == 0) {
		return ch5xx_usb_ep_get_dma_buf(usb, 0);
	}

	return ch5xx_usb_ep_get_dma_buf(usb, ep_hw) + EP_BUF_SIZE;
}

/*
 * Per-endpoint driver state tracked across ISR and thread.
 */
struct ch5xx_ep_data {
	/** Current data toggle for manual-toggle endpoints (EP0,4-7) */
	bool tx_data1;
	bool rx_data1;
};

/*
 * Per-instance driver private data.
 */
struct udc_ch5xx_data {
	struct k_thread thread_data;
	struct k_event events;
	/** Bitmask of OUT endpoints with pending data and no buffer */
	atomic_t out_pending;
	/** Per-endpoint state */
	struct ch5xx_ep_data ep_data[NUM_OF_HW_EPS];
	/** Cached SETUP packet (8 bytes) copied from EP0 RX in ISR */
	uint8_t setup[8];
};

/*
 * Per-instance configuration (from devicetree).
 */
struct udc_ch5xx_config {
	USB_TypeDef *base;
	uint8_t num_eps;
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	void (*irq_enable_func)(const struct device *dev);
	void (*irq_disable_func)(const struct device *dev);
	void (*make_thread)(const struct device *dev);
};

/* ---------- Helpers ---------- */

static inline USB_TypeDef *get_usb_base(const struct device *dev)
{
	const struct udc_ch5xx_config *config = dev->config;

	return config->base;
}

/**
 * Convert endpoint address to a bit index for the out_pending bitmask.
 * OUT endpoints use bits 0-7, IN endpoints use bits 8-15.
 */
static inline uint8_t ep_to_bit(uint8_t ep)
{
	if (USB_EP_DIR_IS_IN(ep)) {
		return USB_EP_GET_IDX(ep) + 8;
	}

	return USB_EP_GET_IDX(ep);
}

/**
 * Flip the manual data toggle for TX on endpoints without auto-toggle.
 */
static inline void toggle_tx(struct udc_ch5xx_data *priv, USB_TypeDef *usb,
			     uint8_t ep_hw)
{
	priv->ep_data[ep_hw].tx_data1 = !priv->ep_data[ep_hw].tx_data1;
	ch5xx_usb_ep_set_tx_tog(usb, ep_hw, priv->ep_data[ep_hw].tx_data1);
}

/**
 * Flip the manual data toggle for RX on endpoints without auto-toggle.
 */
static inline void toggle_rx(struct udc_ch5xx_data *priv, USB_TypeDef *usb,
			     uint8_t ep_hw)
{
	priv->ep_data[ep_hw].rx_data1 = !priv->ep_data[ep_hw].rx_data1;
	ch5xx_usb_ep_set_rx_tog(usb, ep_hw, priv->ep_data[ep_hw].rx_data1);
}

/* ---------- TX (IN) transfer helpers ---------- */

/**
 * Load the next chunk of an IN transfer into the endpoint TX buffer
 * and arm the endpoint for transmission.
 *
 * @return number of bytes loaded, or 0 if nothing to send
 */
static uint16_t ch5xx_load_tx(const struct device *dev, uint8_t ep_hw,
			      struct net_buf *buf)
{
	USB_TypeDef *usb = get_usb_base(dev);
	uint8_t *tx = ep_tx_buf(usb, ep_hw);
	uint16_t len;

	if (buf == NULL || (buf->len == 0 && !udc_ep_buf_has_zlp(buf))) {
		return 0;
	}

	len = MIN(buf->len, EP_BUF_SIZE);
	if (len > 0) {
		memcpy(tx, buf->data, len);
		net_buf_pull(buf, len);
	} else {
		/* ZLP requested */
		udc_ep_buf_clear_zlp(buf);
	}

	ch5xx_usb_ep_set_tx_len(usb, ep_hw, len);
	ch5xx_usb_ep_set_tx_res(usb, ep_hw, UEP_T_RES_ACK);

	LOG_DBG("TX ep_hw %u: %u bytes loaded", ep_hw, len);

	return len;
}

/* ---------- ISR ---------- */

/**
 * Handle SETUP token received on EP0.
 *
 * Copy 8-byte setup packet from RX buffer, NAK both directions on EP0,
 * reset data toggles to DATA1 for the data/status phase, and signal
 * the thread.
 */
static void ch5xx_isr_handle_setup(const struct device *dev)
{
	USB_TypeDef *usb = get_usb_base(dev);
	struct udc_ch5xx_data *priv = udc_get_private(dev);
	uint8_t *rx = ep_rx_buf(usb, 0);

	/* Copy 8 bytes of SETUP data from the EP0 DMA buffer */
	memcpy(priv->setup, rx, 8);

	LOG_HEXDUMP_DBG(priv->setup, 8, "SETUP");

	/* NAK both directions until the stack processes the setup */
	ch5xx_usb_ep_set_rx_res(usb, 0, UEP_R_RES_NAK);
	ch5xx_usb_ep_set_tx_res(usb, 0, UEP_T_RES_NAK);
	ch5xx_usb_ep_set_tx_len(usb, 0, 0);

	/*
	 * After SETUP, data phase starts with DATA1.
	 * EP0 does not have auto-toggle, so set manually.
	 */
	priv->ep_data[0].tx_data1 = true;
	priv->ep_data[0].rx_data1 = true;
	ch5xx_usb_ep_set_tx_tog(usb, 0, true);
	ch5xx_usb_ep_set_rx_tog(usb, 0, true);

	k_event_post(&priv->events, EVT_SETUP);
}

/**
 * Handle IN (TX complete) token in ISR.
 *
 * After a successful IN transaction the host has ACK'd the data.
 * If more data remains in the current net_buf, load the next chunk.
 * Otherwise, complete the transfer.
 */
static void ch5xx_isr_handle_in(const struct device *dev, uint8_t ep_hw)
{
	USB_TypeDef *usb = get_usb_base(dev);
	struct udc_ch5xx_data *priv = udc_get_private(dev);
	uint8_t ep_addr = USB_EP_DIR_IN | ep_hw;
	struct udc_ep_config *cfg = udc_get_ep_cfg(dev, ep_addr);
	struct net_buf *buf;

	if (cfg == NULL) {
		return;
	}

	/* Flip manual toggle for non-auto-toggle endpoints */
	if (!ep_has_auto_tog(ep_hw)) {
		toggle_tx(priv, usb, ep_hw);
	}

	buf = udc_buf_peek(cfg);
	if (buf == NULL) {
		/* No pending transfer - NAK further IN tokens */
		ch5xx_usb_ep_set_tx_res(usb, ep_hw, UEP_T_RES_NAK);
		ch5xx_usb_ep_set_tx_len(usb, ep_hw, 0);
		return;
	}

	if (buf->len > 0 || udc_ep_buf_has_zlp(buf)) {
		/* More data to send - load next chunk */
		ch5xx_load_tx(dev, ep_hw, buf);
	} else {
		/*
		 * Transfer complete. NAK the endpoint, dequeue the buffer,
		 * and submit the completion event. Then try to start the
		 * next queued transfer via thread event.
		 */
		ch5xx_usb_ep_set_tx_res(usb, ep_hw, UEP_T_RES_NAK);
		ch5xx_usb_ep_set_tx_len(usb, ep_hw, 0);

		buf = udc_buf_get(cfg);
		if (buf) {
			udc_submit_ep_event(dev, buf, 0);
		}

		/* Signal thread to check for next queued transfer */
		k_event_post(&priv->events, EVT_XFER_NEXT);
	}
}

/**
 * Handle OUT (RX complete) token in ISR.
 *
 * Copy received data from DMA buffer into the net_buf. If the net_buf
 * is full or we received a short packet, complete the transfer.
 * If no buffer is available, set the pending bit and defer to thread.
 */
static void ch5xx_isr_handle_out(const struct device *dev, uint8_t ep_hw)
{
	USB_TypeDef *usb = get_usb_base(dev);
	struct udc_ch5xx_data *priv = udc_get_private(dev);
	uint8_t ep_addr = USB_EP_DIR_OUT | ep_hw;
	struct udc_ep_config *cfg = udc_get_ep_cfg(dev, ep_addr);
	struct net_buf *buf;
	uint8_t rx_len;
	uint8_t *rx;

	if (cfg == NULL) {
		return;
	}

	rx_len = ch5xx_usb_get_rx_len(usb);
	rx = ep_rx_buf(usb, ep_hw);

	/* Flip manual toggle for non-auto-toggle endpoints */
	if (!ep_has_auto_tog(ep_hw)) {
		toggle_rx(priv, usb, ep_hw);
	}

	buf = udc_buf_peek(cfg);
	if (buf == NULL) {
		/*
		 * No buffer available. NAK the endpoint and defer to thread.
		 * Data remains in the DMA buffer. When a buffer is queued
		 * via ep_enqueue, the thread will be woken to retry.
		 */
		LOG_DBG("OUT ep 0x%02x: no buffer, NAK", ep_addr);
		ch5xx_usb_ep_set_rx_res(usb, ep_hw, UEP_R_RES_NAK);
		atomic_set_bit(&priv->out_pending, ep_to_bit(ep_addr));
		k_event_post(&priv->events, EVT_OUT_PENDING);
		return;
	}

	/* Copy received data into the net_buf */
	uint16_t copy_len = MIN(rx_len, net_buf_tailroom(buf));

	if (copy_len > 0) {
		net_buf_add_mem(buf, rx, copy_len);
	}

	LOG_DBG("OUT ep 0x%02x: %u bytes", ep_addr, copy_len);

	/* Check if transfer is complete (short packet or buffer full) */
	if (rx_len < EP_BUF_SIZE || net_buf_tailroom(buf) == 0) {
		buf = udc_buf_get(cfg);
		if (buf) {
			udc_submit_ep_event(dev, buf, 0);
		}

		/* Signal thread to start next queued transfer if any */
		k_event_post(&priv->events, EVT_XFER_NEXT);
	}

	/* ACK for more data */
	ch5xx_usb_ep_set_rx_res(usb, ep_hw, UEP_R_RES_ACK);
}

/**
 * Handle UIF_TRANSFER interrupt.
 *
 * Read INT_ST to determine the token type and endpoint number,
 * then dispatch to the appropriate handler.
 */
static void ch5xx_isr_transfer(const struct device *dev)
{
	USB_TypeDef *usb = get_usb_base(dev);
	uint8_t int_st = ch5xx_usb_get_int_st(usb);
	uint8_t ep_hw = ch5xx_usb_int_st_ep(int_st);
	uint8_t token = ch5xx_usb_int_st_token(int_st);

	if (ch5xx_usb_int_st_is_setup(int_st) && ep_hw == 0) {
		ch5xx_isr_handle_setup(dev);
	} else if (token == UIS_TOKEN_IN) {
		if (ch5xx_usb_tog_ok(usb)) {
			ch5xx_isr_handle_in(dev, ep_hw);
		}
	} else if (token == UIS_TOKEN_OUT) {
		if (ch5xx_usb_tog_ok(usb)) {
			ch5xx_isr_handle_out(dev, ep_hw);
		}
	}

	/*
	 * Clear UIF_TRANSFER last. The RB_UC_INT_BUSY flag causes
	 * the hardware to auto-NAK while UIF_TRANSFER is set, so
	 * clearing it releases the NAK gate and allows the next
	 * transaction to proceed.
	 */
	ch5xx_usb_clear_int_fg(usb, RB_UIF_TRANSFER);
}

/**
 * Handle bus reset interrupt.
 *
 * Reset all endpoint state, set address to 0, re-enable EP0,
 * and notify the USB stack.
 */
static void ch5xx_isr_bus_reset(const struct device *dev)
{
	USB_TypeDef *usb = get_usb_base(dev);
	struct udc_ch5xx_data *priv = udc_get_private(dev);

	LOG_INF("Bus reset");

	/* Reset address */
	ch5xx_usb_set_addr(usb, 0);

	/* Reset all endpoint data toggles and pending state */
	memset(priv->ep_data, 0, sizeof(priv->ep_data));
	atomic_clear(&priv->out_pending);

	/*
	 * Re-configure EP0:
	 * - Set DMA (already set in init, but be safe)
	 * - TX length 0, NAK both directions initially
	 * - DATA0 toggle
	 */
	ch5xx_usb_ep_set_tx_len(usb, 0, 0);
	ch5xx_usb_ep_set_tx_res(usb, 0, UEP_T_RES_NAK);
	ch5xx_usb_ep_set_rx_res(usb, 0, UEP_R_RES_ACK);
	ch5xx_usb_ep_set_tx_tog(usb, 0, false);
	ch5xx_usb_ep_set_rx_tog(usb, 0, false);

	ch5xx_usb_clear_int_fg(usb, RB_UIF_BUS_RST);

	udc_submit_event(dev, UDC_EVT_RESET, 0);
}

/**
 * Handle suspend interrupt.
 */
static void ch5xx_isr_suspend(const struct device *dev)
{
	USB_TypeDef *usb = get_usb_base(dev);

	LOG_DBG("Suspend");

	ch5xx_usb_clear_int_fg(usb, RB_UIF_SUSPEND);

	udc_set_suspended(dev, true);
	udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
}

/**
 * Main ISR entry point.
 */
static void udc_ch5xx_isr_handler(const struct device *dev)
{
	USB_TypeDef *usb = get_usb_base(dev);
	uint8_t int_fg = ch5xx_usb_get_int_fg(usb);

	if (int_fg & RB_UIF_BUS_RST) {
		ch5xx_isr_bus_reset(dev);
	}

	if (int_fg & RB_UIF_SUSPEND) {
		ch5xx_isr_suspend(dev);
	}

	if (int_fg & RB_UIF_TRANSFER) {
		ch5xx_isr_transfer(dev);
	}

	if (int_fg & RB_UIF_FIFO_OV) {
		LOG_WRN("FIFO overflow");
		ch5xx_usb_clear_int_fg(usb, RB_UIF_FIFO_OV);
	}
}

/* ---------- Thread ---------- */

/**
 * Process pending OUT endpoints that were deferred from ISR because
 * no buffer was available at the time.
 */
static void ch5xx_thread_process_out_pending(const struct device *dev)
{
	USB_TypeDef *usb = get_usb_base(dev);
	struct udc_ch5xx_data *priv = udc_get_private(dev);

	for (uint8_t ep_hw = 0; ep_hw < NUM_OF_HW_EPS; ep_hw++) {
		uint8_t ep_addr = USB_EP_DIR_OUT | ep_hw;
		struct udc_ep_config *cfg;
		struct net_buf *buf;
		uint8_t *rx;
		uint8_t rx_len;

		if (!atomic_test_bit(&priv->out_pending, ep_to_bit(ep_addr))) {
			continue;
		}

		cfg = udc_get_ep_cfg(dev, ep_addr);
		if (cfg == NULL) {
			atomic_clear_bit(&priv->out_pending, ep_to_bit(ep_addr));
			continue;
		}

		buf = udc_buf_peek(cfg);
		if (buf == NULL) {
			/*
			 * Still no buffer. Keep pending bit set so
			 * ep_enqueue will wake us when one arrives.
			 */
			LOG_DBG("OUT ep 0x%02x still no buffer", ep_addr);
			continue;
		}

		/* Read data that has been sitting in the DMA buffer */
		rx_len = ch5xx_usb_get_rx_len(usb);
		rx = ep_rx_buf(usb, ep_hw);

		uint16_t copy_len = MIN(rx_len, net_buf_tailroom(buf));

		if (copy_len > 0) {
			net_buf_add_mem(buf, rx, copy_len);
		}

		LOG_DBG("Thread OUT ep 0x%02x: %u bytes", ep_addr, copy_len);

		atomic_clear_bit(&priv->out_pending, ep_to_bit(ep_addr));

		/* Re-ACK the endpoint */
		ch5xx_usb_ep_set_rx_res(usb, ep_hw, UEP_R_RES_ACK);

		/* Check if transfer is complete */
		if (rx_len < EP_BUF_SIZE || net_buf_tailroom(buf) == 0) {
			buf = udc_buf_get(cfg);
			if (buf) {
				udc_submit_ep_event(dev, buf, 0);
			}
		}
	}
}

/**
 * Try to start the next queued transfer on all endpoints.
 * Called after a transfer completes to kick off any pending work.
 */
static void ch5xx_thread_xfer_next(const struct device *dev)
{
	const struct udc_ch5xx_config *config = dev->config;

	for (uint8_t i = 0; i < config->num_eps; i++) {
		/* Check IN endpoints */
		uint8_t ep_in = USB_EP_DIR_IN | i;
		struct udc_ep_config *cfg_in = udc_get_ep_cfg(dev, ep_in);

		if (cfg_in != NULL && cfg_in->stat.enabled &&
		    !cfg_in->stat.halted) {
			struct net_buf *buf = udc_buf_peek(cfg_in);

			if (buf != NULL && (buf->len > 0 || udc_ep_buf_has_zlp(buf))) {
				ch5xx_load_tx(dev, i, buf);
			}
		}

		/* Check OUT endpoints - just ensure ACK is set */
		uint8_t ep_out = USB_EP_DIR_OUT | i;
		struct udc_ep_config *cfg_out = udc_get_ep_cfg(dev, ep_out);

		if (cfg_out != NULL && cfg_out->stat.enabled &&
		    !cfg_out->stat.halted && i != 0) {
			USB_TypeDef *usb = get_usb_base(dev);

			ch5xx_usb_ep_set_rx_res(usb, i, UEP_R_RES_ACK);
		}
	}
}

/**
 * Main thread handler - processes events posted by the ISR.
 */
static ALWAYS_INLINE void ch5xx_thread_handler(const struct device *const dev)
{
	struct udc_ch5xx_data *priv = udc_get_private(dev);
	uint32_t evt;

	evt = k_event_wait(&priv->events, EVT_ALL, false, K_FOREVER);
	udc_lock_internal(dev, K_FOREVER);

	if (evt & EVT_OUT_PENDING) {
		k_event_clear(&priv->events, EVT_OUT_PENDING);
		ch5xx_thread_process_out_pending(dev);
	}

	if (evt & EVT_SETUP) {
		k_event_clear(&priv->events, EVT_SETUP);
		udc_setup_received(dev, priv->setup);
	}

	if (evt & EVT_XFER_NEXT) {
		k_event_clear(&priv->events, EVT_XFER_NEXT);
		ch5xx_thread_xfer_next(dev);
	}

	udc_unlock_internal(dev);
}

/* ---------- UDC API: Endpoint operations ---------- */

static int udc_ch5xx_ep_enqueue(const struct device *dev,
				struct udc_ep_config *const cfg,
				struct net_buf *const buf)
{
	struct udc_ch5xx_data *priv = udc_get_private(dev);

	udc_buf_put(cfg, buf);

	if (cfg->stat.halted) {
		LOG_DBG("ep 0x%02x halted, queued only", cfg->addr);
		return 0;
	}

	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		/* If this is the first buffer, start the transfer now */
		if (buf == udc_buf_peek(cfg)) {
			uint8_t ep_hw = USB_EP_GET_IDX(cfg->addr);

			ch5xx_load_tx(dev, ep_hw, buf);
		}
	} else {
		if (cfg->addr == USB_CONTROL_EP_OUT) {
			struct udc_buf_info *bi = udc_get_buf_info(buf);

			if (bi->setup) {
				return 0;
			}
		}

		/*
		 * If there's pending OUT data waiting (ISR deferred because
		 * no buffer was available), wake the thread to process it.
		 */
		if (atomic_test_bit(&priv->out_pending,
				    ep_to_bit(cfg->addr))) {
			k_event_post(&priv->events, EVT_OUT_PENDING);
		}
	}

	return 0;
}

static int udc_ch5xx_ep_dequeue(const struct device *dev,
				struct udc_ep_config *const cfg)
{
	USB_TypeDef *usb = get_usb_base(dev);
	struct udc_ch5xx_data *priv = udc_get_private(dev);
	uint8_t ep_hw = USB_EP_GET_IDX(cfg->addr);

	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		ch5xx_usb_ep_set_tx_res(usb, ep_hw, UEP_T_RES_NAK);
		ch5xx_usb_ep_set_tx_len(usb, ep_hw, 0);
	} else {
		atomic_clear_bit(&priv->out_pending, ep_to_bit(cfg->addr));
	}

	udc_ep_cancel_queued(dev, cfg);

	return 0;
}

static int udc_ch5xx_ep_set_halt(const struct device *dev,
				 struct udc_ep_config *const cfg)
{
	USB_TypeDef *usb = get_usb_base(dev);
	uint8_t ep_hw = USB_EP_GET_IDX(cfg->addr);

	LOG_DBG("Set halt ep 0x%02x", cfg->addr);

	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		ch5xx_usb_ep_set_tx_res(usb, ep_hw, UEP_T_RES_STALL);
	} else {
		ch5xx_usb_ep_set_rx_res(usb, ep_hw, UEP_R_RES_STALL);
	}

	if (ep_hw != 0) {
		cfg->stat.halted = true;
	}

	return 0;
}

static int udc_ch5xx_ep_clear_halt(const struct device *dev,
				   struct udc_ep_config *const cfg)
{
	USB_TypeDef *usb = get_usb_base(dev);
	struct udc_ch5xx_data *priv = udc_get_private(dev);
	uint8_t ep_hw = USB_EP_GET_IDX(cfg->addr);
	struct net_buf *buf;

	if (ep_hw == 0) {
		return 0;
	}

	LOG_DBG("Clear halt ep 0x%02x", cfg->addr);

	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		ch5xx_usb_ep_set_tx_res(usb, ep_hw, UEP_T_RES_NAK);
		ch5xx_usb_ep_set_tx_len(usb, ep_hw, 0);

		/* Reset toggle to DATA0 */
		priv->ep_data[ep_hw].tx_data1 = false;
		ch5xx_usb_ep_set_tx_tog(usb, ep_hw, false);
	} else {
		ch5xx_usb_ep_set_rx_res(usb, ep_hw, UEP_R_RES_ACK);
		atomic_clear_bit(&priv->out_pending, ep_to_bit(cfg->addr));

		/* Reset toggle to DATA0 */
		priv->ep_data[ep_hw].rx_data1 = false;
		ch5xx_usb_ep_set_rx_tog(usb, ep_hw, false);
	}

	cfg->stat.halted = false;

	/* Resume any queued IN transfer */
	buf = udc_buf_peek(cfg);
	if (buf != NULL && USB_EP_DIR_IS_IN(cfg->addr)) {
		ch5xx_load_tx(dev, ep_hw, buf);
	}

	return 0;
}

static int udc_ch5xx_ep_enable(const struct device *dev,
			       struct udc_ep_config *const cfg)
{
	USB_TypeDef *usb = get_usb_base(dev);
	struct udc_ch5xx_data *priv = udc_get_private(dev);
	uint8_t ep_hw = USB_EP_GET_IDX(cfg->addr);

	LOG_DBG("Enable ep 0x%02x (hw_ep=%u, type=%u, mps=%u)",
		cfg->addr, ep_hw,
		cfg->attributes & USB_EP_TRANSFER_TYPE_MASK,
		cfg->mps);

	if (ep_hw == 0) {
		/* EP0 is always configured in init/reset, nothing to do */
		return 0;
	}

	/* Enable TX and RX for this endpoint (bidirectional) */
	ch5xx_usb_ep_set_tx_en(usb, ep_hw, true);
	ch5xx_usb_ep_set_rx_en(usb, ep_hw, true);

	/* Configure auto-toggle for EP1-3 */
	if (ep_has_auto_tog(ep_hw)) {
		ch5xx_usb_ep_set_auto_tog(usb, ep_hw, true);
	}

	/* Reset toggles */
	priv->ep_data[ep_hw].tx_data1 = false;
	priv->ep_data[ep_hw].rx_data1 = false;
	ch5xx_usb_ep_set_tx_tog(usb, ep_hw, false);
	ch5xx_usb_ep_set_rx_tog(usb, ep_hw, false);

	/* NAK TX, ACK RX initially */
	ch5xx_usb_ep_set_tx_len(usb, ep_hw, 0);
	ch5xx_usb_ep_set_tx_res(usb, ep_hw, UEP_T_RES_NAK);
	ch5xx_usb_ep_set_rx_res(usb, ep_hw, UEP_R_RES_ACK);

	return 0;
}

static int udc_ch5xx_ep_disable(const struct device *dev,
				struct udc_ep_config *const cfg)
{
	USB_TypeDef *usb = get_usb_base(dev);
	struct udc_ch5xx_data *priv = udc_get_private(dev);
	uint8_t ep_hw = USB_EP_GET_IDX(cfg->addr);

	LOG_DBG("Disable ep 0x%02x", cfg->addr);

	if (ep_hw == 0) {
		return 0;
	}

	/* Disable TX and RX */
	ch5xx_usb_ep_set_tx_en(usb, ep_hw, false);
	ch5xx_usb_ep_set_rx_en(usb, ep_hw, false);

	/* NAK/disable responses */
	ch5xx_usb_ep_set_tx_res(usb, ep_hw, UEP_T_RES_NAK);
	ch5xx_usb_ep_set_rx_res(usb, ep_hw, UEP_R_RES_NAK);

	/* Disable auto-toggle */
	if (ep_has_auto_tog(ep_hw)) {
		ch5xx_usb_ep_set_auto_tog(usb, ep_hw, false);
	}

	/* Clear pending state */
	atomic_clear_bit(&priv->out_pending, ep_to_bit(cfg->addr));

	/* Dequeue any pending buffers */
	udc_ch5xx_ep_dequeue(dev, cfg);

	return 0;
}

/* ---------- UDC API: Controller operations ---------- */

static int udc_ch5xx_set_address(const struct device *dev, const uint8_t addr)
{
	USB_TypeDef *usb = get_usb_base(dev);

	LOG_DBG("Set address %u", addr);

	ch5xx_usb_set_addr(usb, addr);

	return 0;
}

static enum udc_bus_speed udc_ch5xx_device_speed(const struct device *dev)
{
	ARG_UNUSED(dev);

	return UDC_BUS_SPEED_FS;
}

static int udc_ch5xx_host_wakeup(const struct device *dev)
{
	LOG_DBG("Remote wakeup");

	/* CH5xx does not have a dedicated remote wakeup mechanism */
	return -ENOTSUP;
}

static int udc_ch5xx_enable(const struct device *dev)
{
	const struct udc_ch5xx_config *config = dev->config;
	USB_TypeDef *usb = config->base;
	int ret;

	LOG_DBG("Enable controller");

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT,
				     USB_EP_TYPE_CONTROL, EP0_MPS, 0);
	if (ret) {
		LOG_ERR("Failed to enable control OUT endpoint");
		return ret;
	}

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN,
				     USB_EP_TYPE_CONTROL, EP0_MPS, 0);
	if (ret) {
		LOG_ERR("Failed to enable control IN endpoint");
		return ret;
	}

	/* Enable device with internal pull-up */
	ch5xx_usb_dev_enable(usb, true);

	/* Enable interrupts: transfer, bus reset, suspend */
	ch5xx_usb_set_int_en(usb, RB_UIE_TRANSFER | RB_UIE_BUS_RST |
			     RB_UIE_SUSPEND);

	/* EP0: ACK RX, NAK TX, length 0 */
	ch5xx_usb_ep_set_tx_len(usb, 0, 0);
	ch5xx_usb_ep_set_tx_res(usb, 0, UEP_T_RES_NAK);
	ch5xx_usb_ep_set_rx_res(usb, 0, UEP_R_RES_ACK);

	/* Enable IRQ */
	config->irq_enable_func(dev);

	LOG_INF("USB device enabled, waiting for bus reset");

	return 0;
}

static int udc_ch5xx_disable(const struct device *dev)
{
	const struct udc_ch5xx_config *config = dev->config;
	USB_TypeDef *usb = config->base;

	LOG_DBG("Disable controller");

	/* Disable IRQ */
	config->irq_disable_func(dev);

	/* Disable interrupts */
	ch5xx_usb_set_int_en(usb, 0);

	/* Disable pull-up (disconnect from bus) */
	ch5xx_usb_dev_pullup(usb, false);

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_DBG("Failed to disable control OUT endpoint");
	}
	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_DBG("Failed to disable control IN endpoint");
	}

	return 0;
}

static int udc_ch5xx_init(const struct device *dev)
{
	USB_TypeDef *usb = get_usb_base(dev);

	LOG_DBG("Init controller");

	/* Reset the SIE */
	ch5xx_usb_reset_sie(usb);
	ch5xx_usb_clr_all(usb);

	/* Clear all interrupt flags */
	ch5xx_usb_clear_int_fg(usb, RB_UIF_TRANSFER | RB_UIF_BUS_RST |
			       RB_UIF_SUSPEND | RB_UIF_FIFO_OV);

	/* Disable all interrupts initially */
	ch5xx_usb_set_int_en(usb, 0);

	/* Set address to 0 */
	ch5xx_usb_set_addr(usb, 0);

	/*
	 * EP0 DMA buffer is set during driver_preinit via the static
	 * buffer allocation. The DMA address is already configured.
	 */

	return 0;
}

static int udc_ch5xx_shutdown(const struct device *dev)
{
	USB_TypeDef *usb = get_usb_base(dev);

	LOG_DBG("Shutdown");

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_ERR("Failed to disable control OUT endpoint");
	}
	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_ERR("Failed to disable control IN endpoint");
	}

	ch5xx_usb_set_int_en(usb, 0);
	ch5xx_usb_dev_pullup(usb, false);
	ch5xx_usb_reset_sie(usb);
	ch5xx_usb_clr_all(usb);

	return 0;
}

static void udc_ch5xx_lock(const struct device *dev)
{
	udc_lock_internal(dev, K_FOREVER);
}

static void udc_ch5xx_unlock(const struct device *dev)
{
	udc_unlock_internal(dev);
}

/* ---------- UDC API table ---------- */

static const struct udc_api udc_ch5xx_api = {
	.lock = udc_ch5xx_lock,
	.unlock = udc_ch5xx_unlock,
	.device_speed = udc_ch5xx_device_speed,
	.init = udc_ch5xx_init,
	.enable = udc_ch5xx_enable,
	.disable = udc_ch5xx_disable,
	.shutdown = udc_ch5xx_shutdown,
	.set_address = udc_ch5xx_set_address,
	.host_wakeup = udc_ch5xx_host_wakeup,
	.ep_try_config = NULL,
	.ep_enable = udc_ch5xx_ep_enable,
	.ep_disable = udc_ch5xx_ep_disable,
	.ep_set_halt = udc_ch5xx_ep_set_halt,
	.ep_clear_halt = udc_ch5xx_ep_clear_halt,
	.ep_enqueue = udc_ch5xx_ep_enqueue,
	.ep_dequeue = udc_ch5xx_ep_dequeue,
};

/* ---------- Driver preinit (called at boot) ---------- */

static int udc_ch5xx_driver_preinit(const struct device *dev)
{
	const struct udc_ch5xx_config *config = dev->config;
	struct udc_ch5xx_data *priv = udc_get_private(dev);
	struct udc_data *data = dev->data;
	int err;

	k_mutex_init(&data->mutex);
	k_event_init(&priv->events);

	config->make_thread(dev);

	data->caps.rwup = false;
	data->caps.mps0 = UDC_MPS0_64;

	/*
	 * Register EP0 (control endpoint, bidirectional).
	 */
	config->ep_cfg_out[0].caps.out = 1;
	config->ep_cfg_out[0].caps.control = 1;
	config->ep_cfg_out[0].caps.mps = EP0_MPS;
	config->ep_cfg_out[0].addr = USB_CONTROL_EP_OUT;
	err = udc_register_ep(dev, &config->ep_cfg_out[0]);
	if (err) {
		LOG_ERR("Failed to register EP0 OUT");
		return err;
	}

	config->ep_cfg_in[0].caps.in = 1;
	config->ep_cfg_in[0].caps.control = 1;
	config->ep_cfg_in[0].caps.mps = EP0_MPS;
	config->ep_cfg_in[0].addr = USB_CONTROL_EP_IN;
	err = udc_register_ep(dev, &config->ep_cfg_in[0]);
	if (err) {
		LOG_ERR("Failed to register EP0 IN");
		return err;
	}

	/*
	 * Register EP1..N-1 as bidirectional bulk/interrupt endpoints.
	 * Each hardware endpoint supports both IN and OUT simultaneously.
	 */
	for (uint8_t i = 1; i < config->num_eps; i++) {
		config->ep_cfg_in[i].caps.in = 1;
		config->ep_cfg_in[i].caps.bulk = 1;
		config->ep_cfg_in[i].caps.interrupt = 1;
		config->ep_cfg_in[i].caps.iso = 1;
		config->ep_cfg_in[i].caps.mps = EP_BUF_SIZE;
		config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		err = udc_register_ep(dev, &config->ep_cfg_in[i]);
		if (err) {
			LOG_ERR("Failed to register IN ep 0x%02x",
				config->ep_cfg_in[i].addr);
			return err;
		}

		config->ep_cfg_out[i].caps.out = 1;
		config->ep_cfg_out[i].caps.bulk = 1;
		config->ep_cfg_out[i].caps.interrupt = 1;
		config->ep_cfg_out[i].caps.iso = 1;
		config->ep_cfg_out[i].caps.mps = EP_BUF_SIZE;
		config->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		err = udc_register_ep(dev, &config->ep_cfg_out[i]);
		if (err) {
			LOG_ERR("Failed to register OUT ep 0x%02x",
				config->ep_cfg_out[i].addr);
			return err;
		}
	}

	LOG_DBG("Registered %u bidirectional endpoints", config->num_eps);

	return 0;
}

/* ---------- Device instantiation macro ---------- */

#define UDC_CH5XX_DEVICE_DEFINE(n)						\
										\
	static __aligned(4) uint8_t						\
		ep0_buf_##n[EP_BUF_SIZE];					\
	static __aligned(4) uint8_t						\
		epN_buf_##n[DT_INST_PROP(n, num_bidir_endpoints) - 1][128];	\
										\
	K_THREAD_STACK_DEFINE(udc_ch5xx_stack_##n,				\
			      CONFIG_UDC_WCH_CH5XX_STACK_SIZE);			\
										\
	static void udc_ch5xx_thread_##n(void *dev, void *arg1, void *arg2)	\
	{									\
		ARG_UNUSED(arg1);						\
		ARG_UNUSED(arg2);						\
		while (true) {							\
			ch5xx_thread_handler(dev);				\
		}								\
	}									\
										\
	static void udc_ch5xx_make_thread_##n(const struct device *dev)		\
	{									\
		struct udc_ch5xx_data *priv = udc_get_private(dev);		\
										\
		k_thread_create(&priv->thread_data,				\
				udc_ch5xx_stack_##n,				\
				K_THREAD_STACK_SIZEOF(udc_ch5xx_stack_##n),	\
				udc_ch5xx_thread_##n,				\
				(void *)dev, NULL, NULL,			\
				K_PRIO_COOP(CONFIG_UDC_WCH_CH5XX_THREAD_PRIORITY), \
				K_ESSENTIAL,					\
				K_NO_WAIT);					\
		k_thread_name_set(&priv->thread_data, dev->name);		\
	}									\
										\
	static void udc_ch5xx_irq_enable_##n(const struct device *dev)		\
	{									\
		ARG_UNUSED(dev);						\
		IRQ_CONNECT(DT_INST_IRQN(n),					\
			    DT_INST_IRQ(n, priority),				\
			    udc_ch5xx_isr_handler,				\
			    DEVICE_DT_INST_GET(n), 0);				\
		irq_enable(DT_INST_IRQN(n));					\
	}									\
										\
	static void udc_ch5xx_irq_disable_##n(const struct device *dev)		\
	{									\
		ARG_UNUSED(dev);						\
		irq_disable(DT_INST_IRQN(n));					\
	}									\
										\
	static struct udc_ep_config							\
		ep_cfg_out_##n[DT_INST_PROP(n, num_bidir_endpoints)];		\
	static struct udc_ep_config							\
		ep_cfg_in_##n[DT_INST_PROP(n, num_bidir_endpoints)];		\
										\
	static struct udc_ch5xx_data udc_priv_##n;				\
										\
	static void udc_ch5xx_init_buffers_##n(void)				\
	{									\
		USB_TypeDef *usb =						\
			(USB_TypeDef *)DT_INST_REG_ADDR(n);			\
		ch5xx_usb_ep_set_dma(usb, 0, ep0_buf_##n);			\
		for (int i = 0;							\
		     i < (DT_INST_PROP(n, num_bidir_endpoints) - 1);		\
		     i++) {							\
			ch5xx_usb_ep_set_dma(usb, i + 1, epN_buf_##n[i]);	\
		}								\
	}									\
										\
	static int udc_ch5xx_preinit_##n(const struct device *dev)		\
	{									\
		udc_ch5xx_init_buffers_##n();					\
		return udc_ch5xx_driver_preinit(dev);				\
	}									\
										\
	static const struct udc_ch5xx_config udc_cfg_##n = {			\
		.base = (USB_TypeDef *)DT_INST_REG_ADDR(n),			\
		.num_eps = DT_INST_PROP(n, num_bidir_endpoints),		\
		.ep_cfg_in = ep_cfg_in_##n,					\
		.ep_cfg_out = ep_cfg_out_##n,					\
		.irq_enable_func = udc_ch5xx_irq_enable_##n,			\
		.irq_disable_func = udc_ch5xx_irq_disable_##n,			\
		.make_thread = udc_ch5xx_make_thread_##n,			\
	};									\
										\
	static struct udc_data udc_data_##n = {					\
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),		\
		.priv = &udc_priv_##n,						\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n,						\
			      udc_ch5xx_preinit_##n,				\
			      NULL,						\
			      &udc_data_##n,					\
			      &udc_cfg_##n,					\
			      POST_KERNEL,					\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			      &udc_ch5xx_api);

DT_INST_FOREACH_STATUS_OKAY(UDC_CH5XX_DEVICE_DEFINE)

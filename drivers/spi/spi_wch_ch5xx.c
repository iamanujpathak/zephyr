/*
 * drivers/spi/spi_wch_ch5xx.c
 *
 * CH5XX SPI0 driver using internal SPI DMA (half-duplex) with Zephyr SPI_ASYNC_API.
 *
 * - Single-fragment spi_buf_set supported (scatter/gather NOT implemented).
 * - Internal peripheral DMA: program DMA_BEG/DMA_END, set direction, enable DMA.
 * - ISR chains TX -> RX when both tx and rx buffers provided (no runtime threads).
 * - Completion: k_work for async callback, k_sem for synchronous wrapper.
 *
 * Before using:
 *  - Verify register and bit macro names against your hal_ch32fun.h / CH5XXSFR.h
 *  - Ensure device tree node for SPI provides irq (DT_INST_IRQN / priority).
 *  - Adjust compute_spi_divider() to match CH5XX SPI clock formula if needed.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch5xx_spi

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

#include "spi_wch_ch5xx.h"

struct spi_wch_ch5xx_config {
	SPI_TypeDef *regs;
	const struct pinctrl_dev_config *pin_cfg;
	void (*irq_config_func)(const struct device *dev);
};

struct spi_wch_ch5xx_data {
	const struct device *dev;
	const struct spi_config *config;
	const struct spi_buf_set *tx_bufs;
	const struct spi_buf_set *rx_bufs;
#ifdef CONFIG_SPI_ASYNC
	size_t cur_buf_idx;
	size_t cur_buf_len;
	spi_callback_t cb;
	void *userdata;
	struct k_work work;
	struct k_mutex mutex;
	struct k_sem done;
#else
#endif
};

#define tx_buf_count(data) ((data)->tx_bufs ? (data)->tx_bufs->count : 0)
#define rx_buf_count(data) ((data)->rx_bufs ? (data)->rx_bufs->count : 0)

/* Configure SPI (mode, word size, frequency) */
static int spi_wch_ch5xx_configure(const struct device *dev, const struct spi_config *config)
{
	const struct spi_wch_ch5xx_config *cfg = dev->config;
	struct spi_wch_ch5xx_data *data = dev->data;
	SPI_TypeDef *reg = cfg->regs;

	if (!config) {
		return -EINVAL;
	}

	if (data->config == config) {
		return 0;
	}

	/* only master mode is supported as of now */
	if (config->operation & SPI_OP_MODE_SLAVE) {
		return -ENOTSUP;
	}

	if (config->cs.gpio.port) {
		gpio_pin_configure_dt(&config->cs.gpio, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
		gpio_pin_set_dt(&config->cs.gpio, 0);
	}

	ch5xx_spi_set_clock_frequency(reg, CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC, config->frequency);
	ch5xx_spi_set_master_mode(reg, true);
	ch5xx_spi_set_mode(reg, config->operation & SPI_MODE_CPOL,
			   config->operation & SPI_MODE_CPHA);
	ch5xx_spi_set_lsb_first(reg, config->operation & SPI_TRANSFER_LSB);
	ch5xx_spi_set_pin_mode(reg, true, false, true);

	data->config = config;

	return 0;
}

static int spi_wch_ch5xx_transceive_sync(const struct device *dev, const struct spi_config *config,
					 const struct spi_buf_set *tx_bufs,
					 const struct spi_buf_set *rx_bufs)
{
	int err;
	const struct spi_wch_ch5xx_config *cfg = dev->config;
	struct spi_wch_ch5xx_data *data = dev->data;
	uint32_t timeout_ms = 0;
	uint8_t *tx_buf = 0;
	uint8_t *rx_buf = 0;
	uint32_t tx_idx = 0;
	uint32_t tx_len = 0;
	uint32_t rx_idx = 0;
	uint32_t rx_len = 0;

	if (!tx_bufs && !rx_bufs) {
		return -EINVAL;
	}

	err = spi_wch_ch5xx_configure(dev, config);
	if (err) {
		return err;
	}

	data->tx_bufs = tx_bufs;
	data->rx_bufs = rx_bufs;

	if (config->cs.gpio.port) {
		gpio_pin_set_dt(&config->cs.gpio, 1);
		k_sleep(K_USEC(config->cs.delay));
	}

	ch5xx_spi_clear_all(cfg->regs);

	while (tx_idx < tx_buf_count(data) || rx_idx < (rx_buf_count(data))) {
		if (tx_len == 0) {
			if (tx_idx < tx_buf_count(data)) {
				tx_buf = (uint8_t *)data->tx_bufs->buffers[tx_idx].buf;
				tx_len = data->tx_bufs->buffers[tx_idx].len;
				tx_idx++;
			} else {
				tx_buf = NULL;
			}
		}
		if ((rx_len == 0)) {
			if (rx_idx < rx_buf_count(data)) {
				rx_buf = (uint8_t *)data->rx_bufs->buffers[rx_idx].buf;
				rx_len = data->rx_bufs->buffers[rx_idx].len;
				rx_idx++;
			} else {
				rx_buf = NULL;
			}
		}
		if (tx_buf) {
			ch5xx_spi_tx_bytes(cfg->regs, *tx_buf);
			tx_buf++;
		} else {
			ch5xx_spi_tx_bytes(cfg->regs, 0xA5);
		}
		ch5xx_spi_wait_busy(cfg->regs);
		if (rx_buf) {
			*rx_buf = ch5xx_spi_rx_bytes(cfg->regs);
			rx_buf++;
		}
		tx_len--;
		rx_len--;
	}

	return 0;
}

#ifdef CONFIG_SPI_ASYNC

static void spi_wch_ch5xx_complete_transfer(struct spi_wch_ch5xx_data *data)
{
	const struct spi_wch_ch5xx_config *cfg = data->dev->config;
	uint32_t tx_buf_count = tx_buf_count(data);
	uint32_t rx_buf_count = rx_buf_count(data);
	int err = 0;

	if (data->cur_buf_idx < (tx_buf_count + rx_buf_count)) {
		err = -EFAULT;
	}

	if (data->cb) {
		data->cb(data->dev, err, data->userdata);

		if (data->config->cs.gpio.port) {
			gpio_pin_set_dt(&data->config->cs.gpio, 0);
		}

		ch5xx_spi_set_txrx_irq_enable(cfg->regs, false);
		k_mutex_unlock(&data->mutex);
	} else {
		k_sem_give(&data->done);
	}
}

static void spi_wch_ch5xx_work_handler(struct k_work *work)
{
	struct spi_wch_ch5xx_data *data = CONTAINER_OF(work, struct spi_wch_ch5xx_data, work);

	spi_wch_ch5xx_complete_transfer(data);
}

static inline bool get_next_chunk_info(struct spi_wch_ch5xx_data *data, uint8_t **out,
				       uint32_t *len, bool *is_rx)
{
	const struct spi_buf *buf;
	uint32_t tx_buf_count = tx_buf_count(data);
	uint32_t rx_buf_count = rx_buf_count(data);

	while (data->cur_buf_idx < tx_buf_count) {
		buf = &data->tx_bufs->buffers[data->cur_buf_idx];
		if ((buf) && (buf->len > data->cur_buf_len)) {
			*is_rx = false;
			*out = ((uint8_t *)buf->buf) + data->cur_buf_len;
			*len = buf->len - data->cur_buf_len;
			return true;
		}
		data->cur_buf_idx++;
		data->cur_buf_len = 0;
	}

	while (data->cur_buf_idx < (tx_buf_count + rx_buf_count)) {
		buf = &data->rx_bufs->buffers[data->cur_buf_idx - tx_buf_count];
		if ((buf) && (buf->len > data->cur_buf_len)) {
			*is_rx = true;
			*out = ((uint8_t *)buf->buf) + data->cur_buf_len;
			*len = buf->len - data->cur_buf_len;
			return true;
		}
		data->cur_buf_idx++;
		data->cur_buf_len = 0;
	}

	/* If we reach here, we have reached the end of all buffers
	 */
	return false;
}

static void transceive_next_chunk(struct spi_wch_ch5xx_data *data, SPI_TypeDef *regs)
{
	uint8_t *buf;
	uint32_t len;
	bool is_rx;
	bool new_buf = false;

	if (get_next_chunk_info(data, &buf, &len, &is_rx)) {
		while (ch5xx_spi_rx_fifo_ready(regs) && (data->cur_buf_len < len)) {
			*buf++ = ch5xx_spi_read_data(regs);
			data->cur_buf_len++;
		}
	}

	if (get_next_chunk_info(data, &buf, &len, &is_rx)) {
		new_buf = data->cur_buf_len == 0;
		if (new_buf) {
			ch5xx_spi_clear_all(regs);
			ch5xx_spi_set_fifo_dir(regs, is_rx);
			ch5xx_spi_set_txrx_count(regs, len);
		}
		if (!is_rx) {
			while (ch5xx_spi_tx_fifo_ready(regs) && (data->cur_buf_len < len)) {
				ch5xx_spi_write_data(regs, *buf++);
				data->cur_buf_len++;
			}
		}
	} else {
		k_work_submit(&data->work);
	}
}

static void spi_wch_ch5xx_isr(const struct device *dev)
{
	const struct spi_wch_ch5xx_config *cfg = dev->config;
	struct spi_wch_ch5xx_data *data = dev->data;
	uint8_t flags = ch5xx_spi_get_int_flag(cfg->regs);

	ch5xx_spi_clear_irq_flags(cfg->regs);
	transceive_next_chunk(data, cfg->regs);
}

static int spi_wch_ch5xx_transceive_async(const struct device *dev, const struct spi_config *config,
					  const struct spi_buf_set *tx_bufs,
					  const struct spi_buf_set *rx_bufs, spi_callback_t cb,
					  void *userdata)
{
	int err;
	const struct spi_wch_ch5xx_config *cfg = dev->config;
	struct spi_wch_ch5xx_data *data = dev->data;
	uint32_t timeout_ms = 0;

	if (!cb || (!tx_bufs && !rx_bufs)) {
		return -EINVAL;
	}

	/* lock context */
	k_mutex_lock(&data->mutex, K_FOREVER);

	/* clear any stale semaphore */
	k_sem_take(&data->done, K_NO_WAIT);

	err = spi_wch_ch5xx_configure(dev, config);
	if (err) {
		goto done;
	}

	/* update context */
	data->tx_bufs = tx_bufs;
	data->rx_bufs = rx_bufs;
	data->cb = cb;
	data->userdata = userdata;
	data->cur_buf_idx = 0;
	data->cur_buf_len = 0;

	if (config->cs.gpio.port) {
		gpio_pin_set_dt(&config->cs.gpio, 1);
		k_sleep(K_USEC(config->cs.delay));
	}

	ch5xx_spi_set_txrx_irq_enable(cfg->regs, true);
	/* start transfer */
	transceive_next_chunk(data, cfg->regs);

	return err;
}

#endif /* CONFIG_SPI_ASYNC */

static int spi_wch_ch5xx_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_wch_ch5xx_data *data = dev->data;
#ifdef CONFIG_SPI_ASYNC
	spi_wch_ch5xx_complete_transfer(data);
#endif
	return 0;
}

static int spi_wch_ch5xx_init(const struct device *dev)
{
	int err;
	const struct spi_wch_ch5xx_config *cfg = dev->config;
	struct spi_wch_ch5xx_data *data = dev->data;

	/* save reference to device */
	data->dev = dev;

	err = pinctrl_apply_state(cfg->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	ch5xx_spi_set_pin_mode(cfg->regs, true, false, true);

#ifdef CONFIG_SPI_ASYNC
	k_sem_init(&data->done, 0, 1);
	k_mutex_init(&data->mutex);
	k_work_init(&data->work, spi_wch_ch5xx_work_handler);
	cfg->irq_config_func(dev);
#endif

	return 0;
}

/* Driver API binding */
static DEVICE_API(spi, spi_wch_ch5xx_driver_api) = {
	.transceive = spi_wch_ch5xx_transceive_sync,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_wch_ch5xx_transceive_async,
#endif
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif
	.release = spi_wch_ch5xx_release,
};

#ifdef CONFIG_SPI_ASYNC
#define SPI_WCH_CH5XX_IRQ_HANDLER_DECL(idx)                                                        \
	static void spi_wch_ch5xx_irq_config_func_##idx(const struct device *dev);
#define SPI_WCH_CH5XX_IRQ_HANDLER_FUNC(idx) .irq_config_func = spi_wch_ch5xx_irq_config_func_##idx,
#define SPI_WCH_CH5XX_IRQ_HANDLER(idx)                                                             \
	static void spi_wch_ch5xx_irq_config_func_##idx(const struct device *dev)                  \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(idx), DT_INST_IRQ(idx, priority), spi_wch_ch5xx_isr,      \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
		irq_enable(DT_INST_IRQN(idx));                                                     \
	}
#else
#define SPI_WCH_CH5XX_IRQ_HANDLER_DECL(idx)
#define SPI_WCH_CH5XX_IRQ_HANDLER_FUNC(idx)
#define SPI_WCH_CH5XX_IRQ_HANDLER(idx)
#endif

/* Device instantiation */
#define SPI_WCH_CH5XX_INIT(inst)                                                                   \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	SPI_WCH_CH5XX_IRQ_HANDLER_DECL(inst)                                                       \
	static struct spi_wch_ch5xx_data spi_wch_ch5xx_data_##inst;                                \
	static const struct spi_wch_ch5xx_config spi_wch_ch5xx_cfg_##inst = {                      \
		.regs = (SPI_TypeDef *)DT_INST_REG_ADDR(inst),                                     \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                   \
		SPI_WCH_CH5XX_IRQ_HANDLER_FUNC(inst)};                                             \
	DEVICE_DT_INST_DEFINE(inst, &spi_wch_ch5xx_init, NULL, &spi_wch_ch5xx_data_##inst,         \
			      &spi_wch_ch5xx_cfg_##inst, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,    \
			      &spi_wch_ch5xx_driver_api);                                          \
	SPI_WCH_CH5XX_IRQ_HANDLER(inst)

DT_INST_FOREACH_STATUS_OKAY(SPI_WCH_CH5XX_INIT)

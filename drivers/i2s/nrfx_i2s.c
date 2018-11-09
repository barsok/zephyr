/*
 * nrfx_i2s.c
 *
 *  Created on: Nov 7, 2018
 *      Author: barteks
 */

#include <string.h>
#include <dma.h>
#include <i2s.h>
#include <soc.h>
#include <nrfx.h>
#include <nrfx_i2s.h>
#include <stdint.h>

/** @struct i2s_config
 * @brief Interface configuration options.
 *
 * Memory slab pointed to by the mem_slab field has to be defined and
 * initialized by the user. For I2S driver to function correctly number of
 * memory blocks in a slab has to be at least 2 per queue. Size of the memory
 * block should be multiple of frame_size where frame_size = (channels *
 * word_size_bytes). As an example 16 bit word will occupy 2 bytes, 24 or 32
 * bit word will occupy 4 bytes.
 *
 * Please check Zephyr Kernel Primer for more information on memory slabs.
 *
 * @remark When I2S data format is selected parameter channels is ignored,
 * number of words in a frame is always 2.
 *
 * @param word_size Number of bits representing one data word.
 * @param channels Number of words per frame.
 * @param format Data stream format as defined by I2S_FMT_* constants.
 * @param options Configuration options as defined by I2S_OPT_* constants.
 * @param frame_clk_freq Frame clock (WS) frequency, this is sampling rate.
 * @param mem_slab memory slab to store RX/TX data.
 * @param block_size Size of one RX/TX memory block (buffer) in bytes.
 * @param timeout Read/Write timeout. Number of milliseconds to wait in case TX
 *        queue is full, RX queue is empty or one of the special values
 *        K_NO_WAIT, K_FOREVER.
 */

struct nrfx_i2s_cfg {
	uint8_t irq_priority;
	uint8_t sck_pin;
	uint8_t lrck_pin;
	uint8_t mck_pin;
	uint8_t sdout_pin;
	uint8_t sdin_pin;
};

struct direction_config {
	nrfx_i2s_config_t nrfx_config;
	struct k_mem_slab *mem_slab;
	size_t block_size;
	s32_t timeout;
	s32_t state;
	struct i2s_config config_copy;
};

struct nrfx_i2s_data {
	struct direction_config config_tx;
	struct direction_config config_rx;
};

struct stream {
	s32_t state;
	struct k_sem sem;
	u32_t dma_channel;
	struct dma_config dma_cfg;
	struct i2s_config cfg;
	struct ring_buf mem_block_queue;
	void *mem_block;
	bool last_block;
	bool master;
	int (*stream_start)(struct stream *, struct device *dev);
	void (*stream_disable)(struct stream *, struct device *dev);
	void (*queue_drop)(struct stream *);
};




#define NRFX_I2S_DEFAULT_CONFIG_TEST                                   \
{                                                                 \
    .mode         = (nrf_i2s_mode_t)NRFX_I2S_CONFIG_MASTER,       \
    .channels     = (nrf_i2s_channels_t)NRFX_I2S_CONFIG_CHANNELS, \
    .mck_setup    = (nrf_i2s_mck_t)NRFX_I2S_CONFIG_MCK_SETUP,     \
    .ratio        = (nrf_i2s_ratio_t)NRFX_I2S_CONFIG_RATIO,       \
}

#define DEV_CFG(dev) \
	(const struct nrfx_i2s_cfg * const)((dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct nrfx_i2s_data *const)(dev)->driver_data)


void nrfx_i2s_fill_best_clock_settings(nrfx_i2s_config_t *config,
		uint32_t bit_clk_freq)
{
	config->mck_setup = NRF_I2S_MCK_32MDIV125;
	config->ratio = NRF_I2S_RATIO_512X;
}


static int nrfx_i2s_config_for_dir_get(enum i2s_dir dir,
		struct direction_config **config)
{
	struct nrfx_i2s_data *const dev_data = DEV_DATA(dev);

	switch (dir) {
	case I2S_DIR_RX:
		*config = &dev_data->config_rx;
	case I2S_DIR_TX:
		*config = &dev_data->config_tx;
	default:
		LOG_ERR("Either RX or TX direction must be selected");
		return -EINVAL;
	}
	return 0;
}


int nrfx_i2s_configure_direction(struct device *dev, enum i2s_dir dir,
		struct i2s_config *i2s_cfg) {
	const struct nrfx_i2s_cfg *const dev_const_cfg = DEV_CFG(dev);
	struct nrfx_i2s_data *const dev_data = DEV_DATA(dev);
	u32_t bit_clk_freq;
	struct direction_config *dir_config;
	int ret;

	ret = nrfx_i2s_config_for_dir_get(dir, &dir_config);
	if (ret != 0) {
		return ret;
	}

	if (dir_config->state != I2S_STATE_NOT_READY &&
			dir_config->state != I2S_STATE_READY) {
		LOG_ERR("invalid state");
		return -EINVAL;
	}

	dir_config->nrfx_config.sck_pin = dev_const_cfg->sck_pin;
	dir_config->nrfx_config.lrck_pin = dev_const_cfg->lrck_pin;
	dir_config->nrfx_config.mck_pin = dev_const_cfg->mck_pin;
	dir_config->nrfx_config.sdout_pin = dev_const_cfg->sdout_pin;
	dir_config->nrfx_config.sdin_pin = dev_const_cfg->sdin_pin;
	dir_config->nrfx_config.irq_priority = dev_const_cfg->irq_priority;
	dir_config->block_size = i2s_cfg->block_size;
	dir_config->mem_slab = i2s_cfg->mem_slab;
	dir_config->timeout = i2s_cfg->timeout;
	memcpy(&dir_config->config_copy, i2s_cfg, sizeof(dir_config->config_copy));

	if ((i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE) ||
	    (i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE)) {
		dir_config->nrfx_config.mode = NRF_I2S_MODE_SLAVE;
	}
	else {
		dir_config->nrfx_config.mode = NRF_I2S_MODE_MASTER;
	}

	if (i2s_cfg->frame_clk_freq == 0) {
		dir_config->queue_drop(dir_config);
		dir_config->state = I2S_STATE_NOT_READY;
		return 0;
	}

	bit_clk_freq = i2s_cfg->frame_clk_freq *
		       i2s_cfg->word_size * i2s_cfg->channels;

	nrfx_i2s_fill_best_clock_settings(&dir_config->nrfx_config, bit_clk_freq);

	switch (i2s_cfg->word_size) {
	case 8:
		dir_config->nrfx_config.sample_width = NRF_I2S_SWIDTH_8BIT;
		break;
	case 16:
		dir_config->nrfx_config.sample_width = NRF_I2S_SWIDTH_16BIT;
		break;
	case 24:
		dir_config->nrfx_config.sample_width = NRF_I2S_SWIDTH_24BIT;
		break;
	default:
		LOG_ERR("invalid word size");
		return -EINVAL;
	}


	switch (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		dir_config->nrfx_config.alignment = NRF_I2S_ALIGN_LEFT;
		dir_config->nrfx_config.format = NRF_I2S_FORMAT_I2S;
		break;

	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		dir_config->nrfx_config.alignment = NRF_I2S_ALIGN_LEFT;
		dir_config->nrfx_config.format = NRF_I2S_FORMAT_ALIGNED;
		break;

	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		dir_config->nrfx_config.alignment = NRF_I2S_ALIGN_LEFT;
		dir_config->nrfx_config.format = NRF_I2S_FORMAT_ALIGNED;
		break;

	default:
		LOG_ERR("Unsupported I2S data format");
		return -EINVAL;
	}


	dir_config->state = I2S_STATE_READY;
	return 0;
}


struct i2s_config *nrfx_i2s_config_get(struct device *dev, enum i2s_dir dir)
{
	struct direction_config *dir_config;
	int ret;

	ret = nrfx_i2s_config_for_dir_get(dir, &dir_config);
	if (ret != 0) {
		return NULL;
	}

	return &dir_config->config_copy;
}


int (*nrfx_i2s_read)(struct device *dev, void **mem_block, size_t *size);



/**
 * @cond INTERNAL_HIDDEN
 *
 * For internal use only, skip these in public documentation.
 */
struct i2s_driver_api {
	int (*read)(struct device *dev, void **mem_block, size_t *size);
	int (*write)(struct device *dev, void *mem_block, size_t size);
	int (*trigger)(struct device *dev, enum i2s_dir dir,
			enum i2s_trigger_cmd cmd);
};


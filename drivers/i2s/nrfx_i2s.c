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
	s32_t state;
	struct k_sem sem;
	nrfx_i2s_config_t nrfx_config;
	struct k_mem_slab *mem_slab;
	size_t block_size;
	s32_t timeout;
	struct i2s_config config_copy;
	struct ring_buf mem_block_queue;
};

struct nrfx_i2s_data {
	struct direction_config config_tx;
	struct direction_config config_rx;
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


static int nrfx_i2s_dir_config_get(enum i2s_dir dir,
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

	ret = nrfx_i2s_dir_config_get(dir, &dir_config);
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

	/* at the moment I do not see a case with mono sound */
	dir_config->nrfx_config.channels = NRF_I2S_CHANNELS_STEREO;

	dir_config->state = I2S_STATE_READY;
	return 0;
}


struct i2s_config *nrfx_i2s_config_get(struct device *dev, enum i2s_dir dir)
{
	struct direction_config *dir_config;
	int ret;

	ret = nrfx_i2s_dir_config_get(dir, &dir_config);
	if (ret != 0) {
		return NULL;
	}

	return &dir_config->config_copy;
}


int nrfx_i2s_read(struct device *dev, void **mem_block, size_t *size) {
	struct nrfx_i2s_data *const dev_data = DEV_DATA(dev);
	int ret;

	if (dev_data->config_rx.state == I2S_STATE_NOT_READY) {
		LOG_DBG("invalid state");
		return -EIO;
	}

	if (dev_data->config_rx.state != I2S_STATE_ERROR) {
		ret = k_sem_take(&dev_data->config_rx.sem, dev_data->config_rx.timeout);
		if (ret < 0) {
			return ret;
		}
	}

	/* Get data from the beginning of RX queue */
	ret = queue_get(&dev_data->config_rx.mem_block_queue, mem_block, size);
	if (ret < 0) {
		return -EIO;
	}

	return 0;
}


int nrfx_i2s_write(struct device *dev, void *mem_block, size_t size) {
	struct nrfx_i2s_data *const dev_data = DEV_DATA(dev);
	int ret;

	if (dev_data->config_tx.state != I2S_STATE_RUNNING &&
	    dev_data->config_tx.state != I2S_STATE_READY) {
		LOG_DBG("invalid state");
		return -EIO;
	}

	ret = k_sem_take(&dev_data->config_tx.sem, dev_data->config_tx.timeout);
	if (ret < 0) {
		return ret;
	}

	/* Add data to the end of the TX queue */
	queue_put(&dev_data->config_tx.mem_block_queue, mem_block, size);

	return 0;
}


static int nrfx_i2s_trigger(struct device *dev, enum i2s_dir dir,
			     enum i2s_trigger_cmd cmd)
{
	struct nrfx_i2s_data *const dev_data = DEV_DATA(dev);
	struct direction_config *dir_config;
	unsigned int key;
	int ret;

	ret = nrfx_i2s_dir_config_get(dir, &dir_config);
	if (ret != 0) {
		return ret;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (dir_config->state != I2S_STATE_READY) {
			LOG_ERR("START trigger: invalid state %d",
				    dir_config->state);
			return -EIO;
		}

		__ASSERT_NO_MSG(stream->mem_block == NULL);

		ret = dir_config->stream_start(dir_config, dev);
		if (ret < 0) {
			LOG_ERR("START trigger failed %d", ret);
			return ret;
		}

		dir_config->state = I2S_STATE_RUNNING;
		dir_config->last_block = false;
		break;

	case I2S_TRIGGER_STOP:
		key = irq_lock();
		if (dir_config->state != I2S_STATE_RUNNING) {
			irq_unlock(key);
			LOG_ERR("STOP trigger: invalid state");
			return -EIO;
		}
		irq_unlock(key);
		dir_config->stream_disable(dir_config, dev);
		dir_config->queue_drop(dir_config);
		dir_config->state = I2S_STATE_READY;
		dir_config->last_block = true;
		break;

	case I2S_TRIGGER_DRAIN:
		key = irq_lock();
		if (dir_config->state != I2S_STATE_RUNNING) {
			irq_unlock(key);
			LOG_ERR("DRAIN trigger: invalid state");
			return -EIO;
		}
		dir_config->stream_disable(dir_config, dev);
		dir_config->queue_drop(dir_config);
		dir_config->state = I2S_STATE_READY;
		irq_unlock(key);
		break;

	case I2S_TRIGGER_DROP:
		if (stream->state == I2S_STATE_NOT_READY) {
			LOG_ERR("DROP trigger: invalid state");
			return -EIO;
		}
		dir_config->stream_disable(dir_config, dev);
		dir_config->queue_drop(dir_config);
		dir_config->state = I2S_STATE_READY;
		break;

	case I2S_TRIGGER_PREPARE:
		if (dir_config->state != I2S_STATE_ERROR) {
			LOG_ERR("PREPARE trigger: invalid state");
			return -EIO;
		}
		dir_config->state = I2S_STATE_READY;
		dir_config->queue_drop(dir_config);
		break;

	default:
		LOG_ERR("Unsupported trigger command");
		return -EINVAL;
	}

	return 0;
}



static const struct i2s_driver_api i2s_stm32_driver_api = {
	.configure = nrfx_i2s_configure_direction,
	.read = nrfx_i2s_read,
	.write = nrfx_i2s_write,
	.trigger = nrfx_i2s_trigger,
};



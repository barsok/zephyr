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

#define LOG_ERR(...)
#define LOG_DBG(...)

#ifndef uint8_t
#define uint8_t unsigned char
#endif

#define DEV_CFG(dev) \
	(const struct zephyr_i2s_cfg * const)((dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct zephyr_i2s_data *const)(dev)->driver_data)


struct zephyr_i2s_cfg {
	uint8_t sck_pin;
	uint8_t lrck_pin;
	uint8_t mck_pin;
	uint8_t sdout_pin;
	uint8_t sdin_pin;
};

struct queue_item {
	void *data;
	uint32_t size;
};

#define GET_QUEUE_LEN(items) (items + 1)
struct queue {
	struct queue_item *queue_items;
	uint8_t read_idx;
	uint8_t write_idx;
	uint8_t len;
};

struct direction_config {
	s32_t api_state;
	struct k_sem sem;
	struct k_mem_slab *mem_slab;
	s32_t timeout;
	struct i2s_config api_config_copy;
	struct queue mem_block_queue;
	int (*start)(struct direction_config *, struct device *dev);
	void (*stop)(struct direction_config *, struct device *dev);
	void (*drop)(struct direction_config *);
	void (*data_handler)(struct direction_config const * config,
			nrfx_i2s_buffers_t const *p_released, uint32_t status,
			nrfx_i2s_buffers_t *p_new_buffers);
};

struct zephyr_i2s_data {
	nrfx_i2s_config_t nrfx_driver_config;
	size_t block_size;
	uint32_t bit_clk_freq;
	struct direction_config direction_tx;
	struct direction_config direction_rx;
};


int nrfx_agent_add_direction(enum i2s_dir dir, uint32_t *buf,
		int32_t buffer_size);
int nrfx_agent_remove_direction(enum i2s_dir dir);
int nrfx_agent_configure(nrfx_i2s_config_t const *config, struct zephyr_i2s_data const *zephyr_i2s_data);
void nrfx_agent_data_handler(nrfx_i2s_buffers_t const *p_released, uint32_t status);


/*
 *
 *  QUEUE
 *
 */
void queue_init(struct queue *queue, uint8_t len)
{
	queue->read_idx = 0;
	queue->write_idx = 0;
	queue->len = len;
}


static int queue_add(struct queue *queue, void *data, uint32_t size)
{
	uint8_t new_write_idx;

	new_write_idx = queue->write_idx + 1;
	if (new_write_idx >= queue->len) {
		new_write_idx = 0;
	}

	if (new_write_idx == queue->read_idx) {
		/* cannot overwrite unread data */
		return -ENOMEM;
	}

	queue->queue_items[queue->write_idx].data = data;
	queue->queue_items[queue->write_idx].size = size;
	queue->write_idx = new_write_idx;
	return 0;
}


static int queue_fetch(struct queue *queue, void **data, uint32_t *size)
{
	uint8_t new_read_idx;

	if (queue->write_idx == queue->read_idx) {
		/* nothing to read */
		return -ENOMEM;
	}

	*data = queue->queue_items[queue->read_idx].data;
	*size = queue->queue_items[queue->read_idx].size;

	new_read_idx = queue->read_idx + 1;
	if (new_read_idx >= queue->len) {
		new_read_idx = 0;
	}
	queue->read_idx = new_read_idx;

	return 0;
}

/*
 *
 *  CONFIG
 *
 */


void nrfx_i2s_fill_best_clock_settings(nrfx_i2s_config_t *config,
		uint32_t bit_clk_freq)
{
	config->mck_setup = NRF_I2S_MCK_32MDIV125;
	config->ratio = NRF_I2S_RATIO_512X;
}


static int nrfx_i2s_dir_config_get(enum i2s_dir dir,
		struct zephyr_i2s_data *const dev_data,
		struct direction_config **direction_config)
{
	switch (dir) {
	case I2S_DIR_RX:
		*direction_config = &dev_data->direction_rx;
		break;
	case I2S_DIR_TX:
		*direction_config = &dev_data->direction_tx;
		break;
	default:
		LOG_ERR("Either RX or TX direction must be selected");
		return -EINVAL;
	}
	return 0;
}


/*
 *
 *  ZEPHYR API
 *
 */


int nrfx_i2s_api_configure(struct device *dev, enum i2s_dir dir,
		struct i2s_config *i2s_cfg) {

	const struct zephyr_i2s_cfg *const dev_const_cfg = DEV_CFG(dev);
	struct zephyr_i2s_data *const dev_data = DEV_DATA(dev);

	u32_t bit_clk_freq;
	struct direction_config *dir_config;
	int ret;

	ret = nrfx_i2s_dir_config_get(dir, dev_data, &dir_config);
	if (ret != 0) {
		return ret;
	}

	if (dir_config->api_state != I2S_STATE_NOT_READY &&
			dir_config->api_state != I2S_STATE_READY) {
		LOG_ERR("invalid state");
		return -EINVAL;
	}

	/* disable */
	if (i2s_cfg->frame_clk_freq == 0) {
		dir_config->drop(dir_config);
		dir_config->api_state = I2S_STATE_NOT_READY;
		return 0;
	}

	dev_data->nrfx_driver_config.sck_pin = dev_const_cfg->sck_pin;
	dev_data->nrfx_driver_config.lrck_pin = dev_const_cfg->lrck_pin;
	dev_data->nrfx_driver_config.mck_pin = dev_const_cfg->mck_pin;
	dev_data->nrfx_driver_config.sdout_pin = dev_const_cfg->sdout_pin;
	dev_data->nrfx_driver_config.sdin_pin = dev_const_cfg->sdin_pin;

	/* block size is common for tx and rx so raise an error if not the same */
	if ((dev_data->block_size != 0)
		&& (dev_data->block_size != i2s_cfg->block_size)) {
		return -EINVAL;
	}
	dev_data->block_size = i2s_cfg->block_size;
	dir_config->mem_slab = i2s_cfg->mem_slab;
	dir_config->timeout = i2s_cfg->timeout;

	memcpy(&dir_config->api_config_copy, i2s_cfg, sizeof(dir_config->api_config_copy));

	if ((i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE) ||
	    (i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE)) {
		dev_data->nrfx_driver_config.mode = NRF_I2S_MODE_SLAVE;
	}
	else {
		dev_data->nrfx_driver_config.mode = NRF_I2S_MODE_MASTER;
	}

	if (i2s_cfg->frame_clk_freq == 0) {
		dir_config->drop(dir_config);
		dir_config->api_state = I2S_STATE_NOT_READY;
		return 0;
	}

	bit_clk_freq = i2s_cfg->frame_clk_freq *
		       i2s_cfg->word_size * i2s_cfg->channels;

	/* bit freq is common for tx and rx so raise an error if not the same */
	if ((dev_data->bit_clk_freq != 0)
			&& (dev_data->bit_clk_freq != bit_clk_freq)) {
		return -EINVAL;
	}

	nrfx_i2s_fill_best_clock_settings(&dev_data->nrfx_driver_config, bit_clk_freq);

	switch (i2s_cfg->word_size) {
	case 8:
		dev_data->nrfx_driver_config.sample_width = NRF_I2S_SWIDTH_8BIT;
		break;
	case 16:
		dev_data->nrfx_driver_config.sample_width = NRF_I2S_SWIDTH_16BIT;
		break;
	case 24:
		dev_data->nrfx_driver_config.sample_width = NRF_I2S_SWIDTH_24BIT;
		break;
	default:
		LOG_ERR("invalid word size");
		return -EINVAL;
	}


	switch (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		dev_data->nrfx_driver_config.alignment = NRF_I2S_ALIGN_LEFT;
		dev_data->nrfx_driver_config.format = NRF_I2S_FORMAT_I2S;
		break;

	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		dev_data->nrfx_driver_config.alignment = NRF_I2S_ALIGN_LEFT;
		dev_data->nrfx_driver_config.format = NRF_I2S_FORMAT_ALIGNED;
		break;

	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		dev_data->nrfx_driver_config.alignment = NRF_I2S_ALIGN_LEFT;
		dev_data->nrfx_driver_config.format = NRF_I2S_FORMAT_ALIGNED;
		break;

	default:
		LOG_ERR("Unsupported I2S data format");
		return -EINVAL;
	}

	/* at the moment I do not see a case with mono sound */
	dev_data->nrfx_driver_config.channels = NRF_I2S_CHANNELS_STEREO;

	nrfx_agent_configure(&dev_data->nrfx_driver_config, dev_data);

	dir_config->api_state = I2S_STATE_READY;
	return 0;
}


struct i2s_config *nrfx_i2s_config_get(struct device *dev, enum i2s_dir dir)
{
	struct direction_config *dir_config;
	int ret;

	ret = nrfx_i2s_dir_config_get(dir, DEV_DATA(dev), &dir_config);
	if (ret != 0) {
		return NULL;
	}

	return &dir_config->api_config_copy;
}


int nrfx_i2s_read(struct device *dev, void **mem_block, size_t *size) {
	struct zephyr_i2s_data *const dev_data = DEV_DATA(dev);
	int ret;

	if (dev_data->direction_rx.api_state == I2S_STATE_NOT_READY) {
		LOG_DBG("invalid state");
		return -EIO;
	}

	if (dev_data->direction_rx.api_state != I2S_STATE_ERROR) {
		ret = k_sem_take(&dev_data->direction_rx.sem, dev_data->direction_rx.timeout);
		if (ret < 0) {
			return ret;
		}
	}

	ret = queue_fetch(&dev_data->direction_rx.mem_block_queue, mem_block, size);
	if (ret < 0) {
		return -EIO;
	}

	return 0;
}


int nrfx_i2s_write(struct device *dev, void *mem_block, size_t size) {
	struct zephyr_i2s_data *const dev_data = DEV_DATA(dev);
	int ret;

	if (dev_data->direction_tx.api_state != I2S_STATE_RUNNING &&
	    dev_data->direction_tx.api_state != I2S_STATE_READY) {
		LOG_DBG("invalid state");
		return -EIO;
	}

	ret = k_sem_take(&dev_data->direction_tx.sem, dev_data->direction_tx.timeout);
	if (ret < 0) {
		return ret;
	}

	queue_add(&dev_data->direction_tx.mem_block_queue, mem_block, size);

	return 0;
}


static int nrfx_i2s_trigger(struct device *dev, enum i2s_dir dir,
			     enum i2s_trigger_cmd cmd)
{
	struct zephyr_i2s_data *const dev_data = DEV_DATA(dev);
	struct direction_config *dir_config;
	unsigned int key;
	int ret;

	ret = nrfx_i2s_dir_config_get(dir, dev_data, &dir_config);
	if (ret != 0) {
		return ret;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (dir_config->api_state != I2S_STATE_READY) {
			LOG_ERR("START trigger: invalid state %d",
				    dir_config->api_state);
			return -EIO;
		}

		__ASSERT_NO_MSG(stream->mem_block == NULL);

		ret = dir_config->start(dir_config, dev);
		if (ret < 0) {
			LOG_ERR("START trigger failed %d", ret);
			return ret;
		}

		dir_config->api_state = I2S_STATE_RUNNING;
		break;

	case I2S_TRIGGER_STOP:
		key = irq_lock();
		if (dir_config->api_state != I2S_STATE_RUNNING) {
			irq_unlock(key);
			LOG_ERR("STOP trigger: invalid state");
			return -EIO;
		}
		irq_unlock(key);
		dir_config->stop(dir_config, dev);
		dir_config->drop(dir_config);
		dir_config->api_state = I2S_STATE_READY;
		break;

	case I2S_TRIGGER_DRAIN:
		key = irq_lock();
		if (dir_config->api_state != I2S_STATE_RUNNING) {
			irq_unlock(key);
			LOG_ERR("DRAIN trigger: invalid state");
			return -EIO;
		}
		dir_config->stop(dir_config, dev);
		dir_config->drop(dir_config);
		dir_config->api_state = I2S_STATE_READY;
		irq_unlock(key);
		break;

	case I2S_TRIGGER_DROP:
		if (dir_config->api_state == I2S_STATE_NOT_READY) {
			LOG_ERR("DROP trigger: invalid state");
			return -EIO;
		}
		dir_config->stop(dir_config, dev);
		dir_config->drop(dir_config);
		dir_config->api_state = I2S_STATE_READY;
		break;

	case I2S_TRIGGER_PREPARE:
		if (dir_config->api_state != I2S_STATE_ERROR) {
			LOG_ERR("PREPARE trigger: invalid state");
			return -EIO;
		}
		dir_config->api_state = I2S_STATE_READY;
		dir_config->drop(dir_config);
		break;

	default:
		LOG_ERR("Unsupported trigger command");
		return -EINVAL;
	}

	return 0;
}



static const struct i2s_driver_api i2s_stm32_driver_api = {
	.configure = nrfx_i2s_api_configure,
	.read = nrfx_i2s_read,
	.write = nrfx_i2s_write,
	.trigger = nrfx_i2s_trigger,
};


/*
 *
 *  NRFX AGENT
 *
 */

enum nrfx_agent_states {
	NRFX_DRIVER_NOT_INITIALISED,
	NRFX_DRIVER_STOPPED,
	NRFX_DRIVER_RUNNING,
};

struct nrfx_agent_state_actions {
	int (*add_direction)(enum i2s_dir dir, uint32_t *buf, int32_t buffer_size);
	int (*remove_direction)(enum i2s_dir dir);
	int (*configure)(nrfx_i2s_config_t const *config,
			struct zephyr_i2s_data const *zephyr_i2s_data);
	void (*data_handler)(nrfx_i2s_buffers_t const *p_released, uint32_t status);
};


nrfx_i2s_buffers_t nrfx_agent_buffers; /* local pointers to buffers. Indicate which channel is busy */
struct nrfx_agent_state_actions *nrfx_agent_state; /* state of the hardware */
uint32_t nrfx_agent_buffer_size; /* storage of the last rx/tx request */
struct zephyr_i2s_data const *nrfx_agent_its_api; /* pointer to be able to run callbacks */


void nrfx_agent_set_new_state(enum nrfx_agent_states new_state);

bool nrfx_agent_tx_is_active(void) {
	return (nrfx_agent_buffers.p_tx_buffer != NULL) ? true : false;
}

void nrfx_agent_init(void) {
	nrfx_agent_set_new_state(NRFX_DRIVER_NOT_INITIALISED);
}

/* state not initialised */

int not_allowed_add_direction(enum i2s_dir dir, uint32_t *buf, int32_t buffer_size) {
	return -EINVAL;
}

int not_allowed_remove_direction(enum i2s_dir dir) {
	return -EINVAL;
}

void not_allowed_data_handler(nrfx_i2s_buffers_t const *p_released,
		uint32_t status) {
}

void nrfx_agent_data_handler(nrfx_i2s_buffers_t const *p_released, uint32_t status);
int allowed_nrfx_agent_configure(nrfx_i2s_config_t const *config,
		struct zephyr_i2s_data const *zephyr_i2s_data) {
	nrfx_err_t status;

	status = nrfx_i2s_init(config, nrfx_agent_data_handler);
	if (status !=  NRFX_SUCCESS) {
		return -EINVAL;
	}

	nrfx_agent_buffers.p_rx_buffer = NULL;
	nrfx_agent_buffers.p_tx_buffer = NULL;
	nrfx_agent_buffer_size = 0;
	nrfx_agent_its_api = zephyr_i2s_data;

	nrfx_agent_set_new_state(NRFX_DRIVER_STOPPED);
	return 0;
}

struct nrfx_agent_state_actions nrfx_agent_state_not_initialised = {
	not_allowed_add_direction,
	not_allowed_remove_direction,
	allowed_nrfx_agent_configure,
	not_allowed_data_handler,
};

/* shim functions */
int nrfx_agent_add_direction(enum i2s_dir dir, uint32_t *buf,
		int32_t buffer_size) {
	return nrfx_agent_state->add_direction(dir, buf, buffer_size);
}

int nrfx_agent_remove_direction(enum i2s_dir dir) {
	return nrfx_agent_state->remove_direction(dir);
}

int nrfx_agent_configure(nrfx_i2s_config_t const *config,
		struct zephyr_i2s_data const *zephyr_i2s_data) {
	return nrfx_agent_state->configure(config, zephyr_i2s_data);
}

void nrfx_agent_data_handler(nrfx_i2s_buffers_t const *p_released, uint32_t status)
{
	nrfx_agent_state->data_handler(p_released, status);
}

/* state stopped */

int stopped_nrfx_agent_add_direction(enum i2s_dir dir,
		uint32_t *buf,
		int32_t buffer_size) {

	nrfx_err_t status;

	switch (dir) {
	case I2S_DIR_TX:
		if (nrfx_agent_buffers.p_tx_buffer != NULL) {
			return -EIO;
		}
		nrfx_agent_buffers.p_tx_buffer = buf;
		break;
	case I2S_DIR_RX:
		if (nrfx_agent_buffers.p_rx_buffer != NULL) {
			return -EIO;
		}
		nrfx_agent_buffers.p_rx_buffer = buf;
		break;
	default:
		return -EINVAL;
	}

	/* tx request must win over rx request, but no worries -- it can only
	 * be smaller */
	if (nrfx_agent_buffer_size == 0) {
		nrfx_agent_buffer_size = buffer_size;
	}
	else {
		if (dir == I2S_DIR_TX) {
			nrfx_agent_buffer_size = buffer_size;
		}
	}

	status = nrfx_i2s_start(&nrfx_agent_buffers, nrfx_agent_buffer_size, 0);
	if (status != NRFX_SUCCESS) {
		return -EINVAL;
	}

	nrfx_agent_set_new_state(NRFX_DRIVER_RUNNING);
	return 0;
}

void allowed_data_handler(nrfx_i2s_buffers_t const *p_released,
		uint32_t status) {
	struct direction_config const * direction_config;
	nrfx_i2s_buffers_t p_new_buffers;

	p_new_buffers.p_rx_buffer = NULL;
	p_new_buffers.p_tx_buffer = NULL;
	direction_config = &nrfx_agent_its_api->direction_rx;
	direction_config->data_handler(direction_config,
			p_released, status, &p_new_buffers);
	direction_config = &nrfx_agent_its_api->direction_tx;
	direction_config->data_handler(direction_config,
				p_released, status, &p_new_buffers);
	nrfx_i2s_next_buffers_set(&p_new_buffers);
}

void stopped_data_handler(nrfx_i2s_buffers_t const *p_released,
		uint32_t status) {
	if ((status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED)
			== NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) {
		/* fatal error, this should never happen */
		return;
	}

	/* I expect to see remaining data from tx and rx */

}

struct nrfx_agent_state_actions nrfx_agent_state_stopped = {
	stopped_nrfx_agent_add_direction,
	not_allowed_remove_direction,
	allowed_nrfx_agent_configure,
	stopped_data_handler,
};

/* state running */

int running_nrfx_agent_add_direction(uint32_t *buf,
		enum i2s_dir dir,
		int32_t buffer_size) {
	nrfx_agent_set_new_state(NRFX_DRIVER_STOPPED);
	nrfx_i2s_stop();
	return nrfx_agent_add_direction(buf, dir, buffer_size);
}

int running_nrfx_agent_remove_direction(enum i2s_dir dir) {
	nrfx_i2s_stop();

	switch (dir) {
	case I2S_DIR_TX:
		nrfx_agent_buffers.p_tx_buffer = NULL;
		break;
	case I2S_DIR_RX:
		nrfx_agent_buffers.p_rx_buffer = NULL;
		break;
	default:
		return -EINVAL;
	}

	if ((nrfx_agent_buffers. p_tx_buffer == NULL)
		&& (nrfx_agent_buffers.p_rx_buffer == NULL)) {
		nrfx_agent_buffer_size = 0;
		nrfx_agent_set_new_state(NRFX_DRIVER_STOPPED);
		return 0;
	}
	else {
		return stopped_nrfx_agent_add_direction(&nrfx_agent_buffers,
				dir, nrfx_agent_buffer_size);
	}
}

int not_allowed_nrfx_agent_configure(nrfx_i2s_config_t const *config) {
	return -EINVAL;
}

void running_data_handler(nrfx_i2s_buffers_t const *p_released,
		uint32_t status) {
	struct direction_config *direction_config;
	nrfx_i2s_buffers_t p_new_buffers;

	if ((status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED)
			== NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) {

		if (p_released->p_tx_buffer != NULL) {
			nrfx_agent_its_api->direction_tx.data_handler(&nrfx_agent_its_api->direction_tx,
					p_released, status, &p_new_buffers);
		}
	}

	/* I expect to see remaining data from tx and rx */

}

struct nrfx_agent_state_actions nrfx_agent_state_running = {
	running_nrfx_agent_add_direction,
	running_nrfx_agent_remove_direction,
	not_allowed_nrfx_agent_configure,
	running_data_handler,
};

void nrfx_agent_set_new_state(enum nrfx_agent_states new_state) {
	switch (new_state) {
	case NRFX_DRIVER_NOT_INITIALISED:
		nrfx_agent_state = &nrfx_agent_state_not_initialised;
		break;
	case NRFX_DRIVER_STOPPED:
		nrfx_agent_state = &nrfx_agent_state_stopped;
		break;
	case NRFX_DRIVER_RUNNING:
		nrfx_agent_state = &nrfx_agent_state_running;
		break;
	default:
		break;
	}
}

/*
 *
 *   API HANDLERS
 *
 */
#if 0
static void rx_data_handler(struct direction_config *direction_config,
		nrfx_i2s_buffers_t const *p_released, uint32_t status,
		nrfx_i2s_buffers_t *p_released) {

	void *mem_block;

	/* Inform about received data */
	k_sem_give(&direction_config->sem);

	/* Stop reception if we were requested */
	if (direction_config->api_state == I2S_STATE_STOPPING) {
		direction_config->api_state = I2S_STATE_READY;
		goto rx_disable;
	}

	/* Feed more data if necessary */
	if ((status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED)
			== NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) {

		/* Prepare to receive the next data block */
		ret = k_mem_slab_alloc(direction_config->mem_slab, &mem_block, K_NO_WAIT);
		if (ret < 0) {
			direction_config->api_state = I2S_STATE_ERROR;
			goto rx_disable;
		}

		ret = queue_put(&direction_config->mem_block_queue, mem_block,
				nrfx_agent_its_api->block_size);
		if (ret < 0) {
			direction_config->api_state = I2S_STATE_ERROR;
			goto rx_disable;
		}

		p_released->p_rx_buffer = (uint32_t *)mem_block;
	}

	return;

rx_disable:
	//rx_stream_disable(direction_config, dev);
    return;
}
#endif

static void dma_tx_callback(struct direction_config *direction_tx,
		nrfx_i2s_buffers_t const *p_released, uint32_t status,
		nrfx_i2s_buffers_t *p_new_buffers)
{
	struct device *dev = get_dev_from_tx_dma_channel(channel);
	const struct i2s_stm32_cfg *cfg = DEV_CFG(dev);
	struct i2s_stm32_data *const dev_data = DEV_DATA(dev);
	struct stream *stream = &dev_data->tx;
	size_t mem_block_size;
	uint32_t *mem_block;
	int ret;

	k_mem_slab_free(direction_tx->mem_slab, p_released->p_tx_buffer);

	/* Stop transmission if there was an error */
	if (direction_tx->state == I2S_STATE_ERROR) {
		LOG_ERR("TX error detected");
		goto tx_disable;
	}

	/* Stop transmission if we were requested */
	if (direction_tx->last_block) {
		direction_tx->state = I2S_STATE_READY;
		goto tx_disable;
	}



	if (mem_block_size !=
			CONTAINER_OF(direction_tx, struct zephyr_i2s_data, direction_tx)->block_size) {
		direction_tx->state = I2S_STATE_ERROR;
		goto tx_disable;
	}

	/* Prepare to send the next data block */
	ret = queue_fetch(&direction_tx->mem_block_queue, &mem_block,
			&mem_block_size);
	if (ret < 0) {
		if (direction_tx->state == I2S_STATE_STOPPING) {
			direction_tx->state = I2S_STATE_READY;
		} else {
			direction_tx->state = I2S_STATE_ERROR;
		}
		goto tx_disable;
	}
	k_sem_give(&direction_tx->sem);

	p_new_buffers->p_tx_buffer = mem_block;

	return;

tx_disable:
	tx_stream_disable(direction_tx, dev);
}


static int nrfx_i2s_initialize(struct device *dev)
{
	const struct zephyr_i2s_cfg *cfg = DEV_CFG(dev);
	struct zephyr_i2s_data *const dev_data = DEV_DATA(dev);
	int ret, i;

	k_sem_init(&dev_data->direction_rx.sem, 0, CONFIG_I2S_STM32_RX_BLOCK_COUNT);
	k_sem_init(&dev_data->direction_tx.sem, CONFIG_I2S_STM32_TX_BLOCK_COUNT,
		   CONFIG_I2S_STM32_TX_BLOCK_COUNT);

	nrfx_agent_init();

	LOG_INF("%s inited", dev->config->name);

	return 0;
}

#if 0
static int rx_stream_start(struct stream *stream, struct device *dev)
{
	const struct i2s_stm32_cfg *cfg = DEV_CFG(dev);
	struct i2s_stm32_data *const dev_data = DEV_DATA(dev);
	int ret;

	ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block,
			       K_NO_WAIT);
	if (ret < 0) {
		return ret;
	}

	if (stream->master) {
		LL_I2S_SetTransferMode(cfg->i2s, LL_I2S_MODE_MASTER_RX);
	} else {
		LL_I2S_SetTransferMode(cfg->i2s, LL_I2S_MODE_SLAVE_RX);
	}

	/* remember active RX DMA channel (used in callback) */
	active_dma_rx_channel[stream->dma_channel] = dev;

	ret = start_dma(dev_data->dev_dma, stream->dma_channel,
			&stream->dma_cfg,
			(void *)LL_SPI_DMA_GetRegAddr(cfg->i2s),
			stream->mem_block,
			stream->cfg.block_size);
	if (ret < 0) {
		LOG_ERR("Failed to start RX DMA transfer: %d", ret);
		return ret;
	}

	LL_I2S_EnableDMAReq_RX(cfg->i2s);

	LL_I2S_EnableIT_ERR(cfg->i2s);
	LL_I2S_Enable(cfg->i2s);

	return 0;
}
#endif

static int tx_direction_start(struct direction_config const *direction_tx) {
	uint32_t *mem_block;
	size_t mem_block_size;
	int ret;

	ret = queue_fetch(&direction_tx->mem_block_queue, &mem_block,
			&mem_block_size);
	if (ret < 0) {
		return ret;
	}
	k_sem_give(&direction_tx->sem);

	nrfx_agent_add_direction(I2S_DIR_TX, mem_block, mem_block_size);

	return 0;
}

#if 0
static void rx_stream_disable(struct stream *stream, struct device *dev)
{
	const struct i2s_stm32_cfg *cfg = DEV_CFG(dev);
	struct i2s_stm32_data *const dev_data = DEV_DATA(dev);
	struct device *dev_dma = dev_data->dev_dma;

	LL_I2S_DisableDMAReq_RX(cfg->i2s);
	LL_I2S_DisableIT_ERR(cfg->i2s);

	dma_stop(dev_dma, stream->dma_channel);
	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, &stream->mem_block);
		stream->mem_block = NULL;
	}

	LL_I2S_Disable(cfg->i2s);

	active_dma_rx_channel[stream->dma_channel] = NULL;
}

static void tx_stream_disable(struct stream *stream, struct device *dev)
{
	const struct i2s_stm32_cfg *cfg = DEV_CFG(dev);
	struct i2s_stm32_data *const dev_data = DEV_DATA(dev);
	struct device *dev_dma = dev_data->dev_dma;

	LL_I2S_DisableDMAReq_TX(cfg->i2s);
	LL_I2S_DisableIT_ERR(cfg->i2s);

	dma_stop(dev_dma, stream->dma_channel);
	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, &stream->mem_block);
		stream->mem_block = NULL;
	}

	LL_I2S_Disable(cfg->i2s);

	active_dma_tx_channel[stream->dma_channel] = NULL;
}

static void rx_queue_drop(struct stream *stream)
{
	size_t size;
	void *mem_block;

	while (queue_get(&stream->mem_block_queue, &mem_block, &size) == 0) {
		k_mem_slab_free(stream->cfg.mem_slab, &mem_block);
	}

	k_sem_reset(&stream->sem);
}

static void tx_queue_drop(struct stream *stream)
{
	size_t size;
	void *mem_block;
	unsigned int n = 0;

	while (queue_get(&stream->mem_block_queue, &mem_block, &size) == 0) {
		k_mem_slab_free(stream->cfg.mem_slab, &mem_block);
		n++;
	}

	for (; n > 0; n--) {
		k_sem_give(&stream->sem);
	}
}

#endif


#ifdef CONFIG_I2S_NRFX
static struct device DEVICE_NAME_GET(i2s0);

static void i2s_stm32_irq_config_func_1(struct device *dev);

static const struct zephyr_i2s_cfg zephyr_i2s_cfg_0 = {
		.sck_pin = DT_I2S0_SCK_PIN,
		.lrck_pin =  DT_I2S0_LRCK_PIN,
		.mck_pin = DT_I2S0_MCK_PIN,
		.sdout_pin = DT_I2S0_SDOUT_PIN,
		.sdin_pin = DT_I2S0_SDIN_PIN,
};

struct queue_item rx_1_ring_buf[CONFIG_I2S_STM32_RX_BLOCK_COUNT + 1];
struct queue_item tx_1_ring_buf[CONFIG_I2S_STM32_TX_BLOCK_COUNT + 1];

static struct i2s_stm32_data i2s_stm32_data_1 = {
	.dma_name = I2S1_DMA_NAME,
	.rx = {
		.dma_channel = I2S1_DMA_CHAN_RX,
		.dma_cfg = {
			.block_count = 1,
			.dma_slot = I2S1_DMA_SLOT_RX,
			.channel_direction = PERIPHERAL_TO_MEMORY,
			.source_data_size = 1,  /* 16bit default */
			.dest_data_size = 1,    /* 16bit default */
			.source_burst_length = 0, /* SINGLE transfer */
			.dest_burst_length = 1,
			.dma_callback = dma_rx_callback,
		},
		.stream_start = rx_stream_start,
		.stream_disable = rx_stream_disable,
		.queue_drop = rx_queue_drop,
		.mem_block_queue.buf = rx_1_ring_buf,
		.mem_block_queue.len = ARRAY_SIZE(rx_1_ring_buf),
	},
	.tx = {
		.dma_channel = I2S1_DMA_CHAN_TX,
		.dma_cfg = {
			.block_count = 1,
			.dma_slot = I2S1_DMA_SLOT_TX,
			.channel_direction = MEMORY_TO_PERIPHERAL,
			.source_data_size = 1,  /* 16bit default */
			.dest_data_size = 1,    /* 16bit default */
			.source_burst_length = 1,
			.dest_burst_length = 0, /* SINGLE transfer */
			.dma_callback = dma_tx_callback,
		},
		.stream_start = tx_stream_start,
		.stream_disable = tx_stream_disable,
		.queue_drop = tx_queue_drop,
		.mem_block_queue.buf = tx_1_ring_buf,
		.mem_block_queue.len = ARRAY_SIZE(tx_1_ring_buf),
	},
};
DEVICE_AND_API_INIT(i2s_stm32_1, CONFIG_I2S_1_NAME, &i2s_stm32_initialize,
		    &i2s_stm32_data_1, &i2s_stm32_config_1, POST_KERNEL,
		    CONFIG_I2S_INIT_PRIORITY, &i2s_stm32_driver_api);

static void i2s_stm32_irq_config_func_1(struct device *dev)
{
	IRQ_CONNECT(CONFIG_I2S_1_IRQ, CONFIG_I2S_1_IRQ_PRI, i2s_stm32_isr,
		    DEVICE_GET(i2s_stm32_1), 0);
	irq_enable(CONFIG_I2S_1_IRQ);
}

#endif /* CONFIG_I2S_1 */

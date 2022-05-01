#include "ipc.h"

#include <kernel.h>

#include <device.h>
#include <devicetree.h>
#include <drivers/uart.h>

#if defined(CONFIG_UART_IPC_DEBUG_GPIO)
#	include <stm32f4xx_hal_gpio.h>
#	include <stm32f4xx_ll_gpio.h>
#endif

#if defined(CONFIG_UART_IPC)

#include <logging/log.h>
LOG_MODULE_REGISTER(ipc, LOG_LEVEL_WRN);

// TODO monitor usage using CONFIG_MEM_SLAB_TRACE_MAX_UTILIZATION

// config
#define IPC_UART_RX_TIMEOUT_MS 1U

#if defined(CONFIG_UART_IPC_DEBUG_GPIO)

#	define DBG_PIN_PORT GPIOC
#	define DBG_PIN_1 GPIO_PIN_8
#	define DBG_PIN_2 GPIO_PIN_9
#	define DBG_PIN_3 GPIO_PIN_10
#	define DBG_PIN_4 GPIO_PIN_11
#	define DBG_PIN_5 GPIO_PIN_12

#	define __DEBUG_RX() LL_GPIO_TogglePin(GPIOC, DBG_PIN_1)
#	define __DEBUG_RX_HANDLER_ENTER() LL_GPIO_SetOutputPin(GPIOC, DBG_PIN_2)
#	define __DEBUG_RX_HANDLER_EXIT() LL_GPIO_ResetOutputPin(GPIOC, DBG_PIN_2)
#	define __DEBUG_RX_FRAME_READY() LL_GPIO_TogglePin(GPIOC, DBG_PIN_3)

#else

#	define __DEBUG_RX() 
#	define __DEBUG_RX_HANDLER_ENTER()
#	define __DEBUG_RX_HANDLER_EXIT()
#	define __DEBUG_RX_FRAME_READY()

#endif



// drivers
#define IPC_UART_NODE DT_ALIAS(ipc_uart)
static const struct device *uart_dev = DEVICE_DT_GET(IPC_UART_NODE);

// TODO check that device is compatible UART + DMA
#if !DT_NODE_HAS_STATUS(IPC_UART_NODE, okay)
#	error "ipc_uart device node not found"
#endif

// SHARED
static K_MEM_SLAB_DEFINE(frames_slab, IPC_FRAME_SIZE, 
			 CONFIG_UART_IPC_FRAME_BUFFER_COUNT, 4);


static inline int alloc_frame(ipc_frame_t **p_frame) {
	return k_mem_slab_alloc(&frames_slab, (void **)p_frame, K_NO_WAIT);
}

static inline void free_frame(ipc_frame_t **p_frame) {
	if (*p_frame != NULL) {
		k_mem_slab_free(&frames_slab, (void **)p_frame);
		*p_frame = NULL;
	}
}

// RX
#if defined(CONFIG_UART_IPC_RX) || defined(CONFIG_UART_IPC_FULL)

static uint8_t double_buffer[2][CONFIG_UART_IPC_DMA_BUF_SIZE];
static uint8_t *next_buffer = double_buffer[1];

static K_FIFO_DEFINE(rx_fifo);

static struct k_msgq *rx_mxgq = NULL;

static uint32_t rx_seq = 0U;

typedef enum {
	/**
	 * @brief Waiting for the first byte of the start delimiter
	 */
	PARSING_CATCH_FRAME = 0, 

	/**
	 * @brief Parsing the sfd
	 */
	PARSING_FRAME_START_DELIMITER,

	/**
	 * @brief Parsing the data,
	 * waiting for the first byte of the end delimiter
	 */
	PARSING_FRAME_DATA,
} parsing_state_t;

// parsing context
static struct {
	parsing_state_t state;
	size_t remaining;
	union {
		ipc_frame_t *frame;
		uint8_t *raw;
	};
	size_t filling;
} ctx = {
	.state = PARSING_CATCH_FRAME,
	.remaining = 0U,
	.filling = 0U
};

#endif /* UART_IPC_RX */

// TX
#if defined(CONFIG_UART_IPC_TX) || defined(CONFIG_UART_IPC_FULL)
static K_SEM_DEFINE(tx_sem, 1, 1);
K_FIFO_DEFINE(tx_fifo);

static uint32_t tx_seq = 0U;

static inline void queue_tx_frame(ipc_frame_t *frame)
{
	__ASSERT(frame != NULL, "frame is NULL");

	k_fifo_put(&tx_fifo, frame);
}
#endif /* UART_IPC_RX */

// statistics and debug to debug functions in ISR context especially
// TODO

#if defined(CONFIG_UART_IPC_RX) || defined(CONFIG_UART_IPC_FULL)
int ipc_attach_rx_msgq(struct k_msgq *msgq)
{
	rx_mxgq = msgq;

	return 0;
}

/**
 * @brief Copy data from the buffer to the context frame
 * 
 * @param data ²
 * @param len 
 */
static inline void copy(const uint8_t *data, size_t len)
{
	memcpy(ctx.raw + ctx.filling, data, len);
	ctx.filling += len;
}

/**
 * @brief This function is called from an ISR, so don't do logging in it at all
 * 
 * @param dev 
 * @param evt 
 * @param user_data 
 */
static void handle_received_chunk(const uint8_t *data, size_t size)
{
	__DEBUG_RX_HANDLER_ENTER();

	while (size > 0) {
		switch (ctx.state) {
		case PARSING_CATCH_FRAME:
		{
			uint8_t *const p = (uint8_t *)
				memchr(data, IPC_START_FRAME_DELIMITER_BYTE, size);
			if (p == NULL) {
				/* first byte of the start frame
				 * delimiter not found in the chuck
				 */
				goto discard_rest;
			}

			if (ctx.frame == NULL) {
				if (alloc_frame(&ctx.frame) != 0) {
					goto discard_rest;
				}
			}

			size -= (p - data) + 1U;
			data = p + 1U;

			ctx.remaining = IPC_START_FRAME_DELIMITER_SIZE - 1U;
			ctx.state = PARSING_FRAME_START_DELIMITER;
			break;
		}
		case PARSING_FRAME_START_DELIMITER:
		{
			while (size > 0) {
				if (*data != IPC_START_FRAME_DELIMITER_BYTE) {
					ctx.state = PARSING_CATCH_FRAME;
					break;
				}
				data++;
				size--;
				if (--ctx.remaining == 0U) {
					ctx.remaining = IPC_FRAME_SIZE
						- IPC_START_FRAME_DELIMITER_SIZE;
					ctx.state = PARSING_FRAME_DATA;
					ctx.filling = IPC_START_FRAME_DELIMITER_SIZE;
					break;
				}
			}
			break;
		}
		case PARSING_FRAME_DATA:
		{
			size_t tocopy = (ctx.remaining < size) ? ctx.remaining : size;
			copy(data, tocopy);
			data += tocopy;
			size -= tocopy;
			ctx.remaining -= tocopy;

			if (ctx.remaining == 0U) {
				/* Pass the detected frame to the processing thread by
				* the FIFO
				* Note: As queuing a buffer to a FIFO requires
				*  to write an address at the beginning, the start
				*  frame delimiter is not forwarded.
				*  However we don't need it anymore as it has been already
				*  checked in this function.
				*/
				k_fifo_put(&rx_fifo, ctx.frame);

				ctx.frame = NULL;
				ctx.state = PARSING_CATCH_FRAME;
				__DEBUG_RX_FRAME_READY();
			}
			break;
		}
		}
	}

	__DEBUG_RX_HANDLER_EXIT();
	return;

discard_rest:
	ctx.state = PARSING_CATCH_FRAME;
	__DEBUG_RX_HANDLER_EXIT();
	return;
}

#endif

static void uart_callback(const struct device *dev,
			  struct uart_event *evt,
			  void *user_data)
{
	switch (evt->type) {
#if defined(CONFIG_UART_IPC_TX) || defined(CONFIG_UART_IPC_FULL)
	case UART_TX_DONE:
		free_frame((ipc_frame_t **)&evt->data);
		k_sem_give(&tx_sem);
		break;
	case UART_TX_ABORTED:
		break;
#endif /* CONFIG_UART_IPC_TX */

#if defined(CONFIG_UART_IPC_RX) || defined(CONFIG_UART_IPC_FULL)
	case UART_RX_RDY:
	{
		__DEBUG_RX();
		handle_received_chunk(evt->data.rx.buf + evt->data.rx.offset,
				       evt->data.rx.len);
		break;
	}
	case UART_RX_BUF_REQUEST:
	{
		uart_rx_buf_rsp(dev, next_buffer, CONFIG_UART_IPC_DMA_BUF_SIZE);
		break;
	}
	case UART_RX_BUF_RELEASED:
	{
		next_buffer = evt->data.rx_buf.buf;
		break;
	}
	case UART_RX_DISABLED:
	{
		break;
	}
	case UART_RX_STOPPED:
	{
		break;
	}
#endif /* CONFIG_UART_IPC_TX */
	default:
		break;
	}
}

extern uint32_t crc_calculate32(uint32_t *buf,
				size_t len);

static inline uint32_t ipc_frame_crc32(ipc_frame_t *frame)
{
	uint32_t *const start = (uint32_t *)&frame->seq;
	size_t len = sizeof(frame->data) + sizeof(frame->seq); // multiple of 4

	return crc_calculate32(start, len >> 2);
}

enum {
	IPC_DIRECTION_RX = 0,
	IPC_DIRECTION_TX = 1,
};

static void ipc_log_frame(const ipc_frame_t *frame, uint8_t direction)
{
	const char *dir = "TX";
	uint32_t start_delimiter = frame->start_delimiter;
	if (direction == IPC_DIRECTION_RX) {
		/* set it back for display */
		start_delimiter = IPC_START_FRAME_DELIMITER;
		dir = "RX";
	}

	LOG_INF("%s IPC frame: %u B seq = %x data size = %u sfd = %x crc32=%x", dir,
		IPC_FRAME_SIZE, frame->seq, frame->data.size, start_delimiter,
		frame->crc32);
	LOG_HEXDUMP_DBG(frame->data.buf, frame->data.size, "IPC frame data");
}

// thread
static void ipc_thread(void *_a, void *_b, void *_c);

K_THREAD_DEFINE(ipc_thread_id, CONFIG_UART_IPC_STACK_SIZE, ipc_thread,
		NULL, NULL, NULL, K_PRIO_PREEMPT(8), 0, 0);

#if defined(CONFIG_UART_IPC_FULL)
static union {
	struct {
		struct k_poll_event rx_ev;
		struct k_poll_event tx_ev;
	};

	struct k_poll_event array[2];
} events = {
	.rx_ev = K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_FIFO_DATA_AVAILABLE,
					  K_POLL_MODE_NOTIFY_ONLY,
					  &rx_fifo),
	.tx_ev = K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_FIFO_DATA_AVAILABLE,
					  K_POLL_MODE_NOTIFY_ONLY,
					  &tx_fifo),
};
#endif

#if defined(CONFIG_UART_IPC_RX) || defined(CONFIG_UART_IPC_FULL)
static int handle_rx_frame(ipc_frame_t *frame)
{
	int ret = -1;
	
	/* verify length */
	if(frame->data.size > IPC_MAX_DATA_SIZE) {
		LOG_ERR("Invalid data size = %u", frame->data.size);
		goto exit;
	}

	/* verify crc32 */
	const uint32_t crc32 = ipc_frame_crc32(frame);
	if (crc32 == frame->crc32) {
		/* dispatch frame to application */
		if (rx_mxgq != NULL) {
			ret = k_msgq_put(rx_mxgq, frame, K_NO_WAIT);
			if (ret != 0) {
				LOG_WRN("Provided user msgq is full, dropping frame %p",
					frame);
			}
		} else {
			LOG_WRN("No user msgq provided, dropping frame %p", frame);
		}
	} else {
		LOG_ERR("CRC32 mismatch: %x != %x", crc32, frame->crc32);
		goto exit;
	}

	/* compare and update sequence number */
	if (frame->seq > rx_seq + 1) {
		LOG_WRN("Seq gap %u -> %u, %u frames lost", rx_seq,
			frame->seq, frame->seq - rx_seq - 1);
	} else if (frame->seq < rx_seq) {
		LOG_WRN("Seq rollback from %u to %u, peer probably reseted",
			rx_seq, frame->seq);
	}
	rx_seq = frame->seq;

	/* show if valid */
	ipc_log_frame(frame, IPC_DIRECTION_RX);

exit:
	free_frame(&frame);

	return 0;
}
#endif /* CONFIG_UART_IPC_RX */

#if defined(CONFIG_UART_IPC_TX) || defined(CONFIG_UART_IPC_FULL)
static void prepare_frame(ipc_frame_t *frame, uint32_t sequence_number)
{
	frame->start_delimiter = IPC_START_FRAME_DELIMITER;

	/* set sequence number */
	frame->seq = tx_seq;

	/* final check on data size */
	if (frame->data.size > IPC_MAX_DATA_SIZE) {
		LOG_ERR("IPC frame data size too big = %d", frame->data.size);
		frame->data.size = IPC_MAX_DATA_SIZE;
	}

	/* padding */
	memset(&frame->data.buf[frame->data.size], 0x00,
	       IPC_MAX_DATA_SIZE - frame->data.size);
	
	/* calculate crc32 */
	frame->crc32 = ipc_frame_crc32(frame);
}

static int handle_tx_frame(ipc_frame_t *frame)
{
	int ret;

	/* prepare IPC frame, sfd, efd, crc32, ... */
	prepare_frame(frame, tx_seq);

	/* wait for tx finish */
	k_sem_take(&tx_sem, K_FOREVER);

	/* send frame */
	ret = uart_tx(uart_dev, (const uint8_t *)frame,
		      IPC_FRAME_SIZE, 0);
	if (ret == 0) {
		/* show frame being sent */
		ipc_log_frame(frame, IPC_DIRECTION_TX);

		/* increment sequence number */
		tx_seq++;
	} else {
		LOG_ERR("uart_tx failed = %d", ret);

		/* retry on failure */
		queue_tx_frame(frame);
	}

	return ret;
}
#endif /* CONFIG_UART_IPC_TX */

static void ipc_thread(void *_a, void *_b, void *_c)
{
	int ret;
	ipc_frame_t *frame = NULL;

	if (device_is_ready(uart_dev) == false) {
		LOG_ERR("IPC UART device not ready = %d", 0);
		return;
	}

	ret = uart_callback_set(uart_dev, uart_callback, NULL);
	if (ret != 0) {
		LOG_ERR("uart_callback_set() failed %d", ret);
		return;
	}

#if defined(CONFIG_UART_IPC_DEBUG_GPIO)

	__HAL_RCC_GPIOC_CLK_ENABLE();

	static GPIO_InitTypeDef gpio_initstruct = {
		.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12,
		.Mode = GPIO_MODE_OUTPUT_PP,
		.Pull = GPIO_PULLUP,
		.Speed = GPIO_SPEED_FREQ_HIGH,
	};

	HAL_GPIO_Init(GPIOC, &gpio_initstruct);

	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8 | LL_GPIO_PIN_9 |
			       LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12);
#endif

#if defined(CONFIG_UART_IPC_RX) || defined(CONFIG_UART_IPC_FULL)
	ret = uart_rx_enable(uart_dev,
			     double_buffer[0],
			     CONFIG_UART_IPC_DMA_BUF_SIZE,
			     IPC_UART_RX_TIMEOUT_MS);
	if (ret != 0) {
		LOG_ERR("uart_rx_enable() failed %d", ret);
		return;
	}
#endif /* CONFIG_UART_IPC_RX */

	for (;;) {
#if defined(CONFIG_UART_IPC_FULL)
		ret = k_poll(events.array, ARRAY_SIZE(events.array), K_FOREVER);
		if (ret >= 0) {
			if (events.rx_ev.state == K_POLL_STATE_FIFO_DATA_AVAILABLE) {
				frame = (ipc_frame_t *)k_fifo_get(&rx_fifo, K_NO_WAIT);
				__ASSERT(frame != NULL, "RX frame is NULL");
				handle_rx_frame(frame);
			}
			events.rx_ev.state = K_POLL_STATE_NOT_READY;
			if (events.tx_ev.state == K_POLL_STATE_FIFO_DATA_AVAILABLE) {
				frame = (ipc_frame_t *)k_fifo_get(&tx_fifo, K_NO_WAIT);
				__ASSERT(frame != NULL, "TX frame is NULL");
				handle_tx_frame(frame);
			}
			events.tx_ev.state = K_POLL_STATE_NOT_READY;
		} else {
			LOG_ERR("k_poll() failed %d", ret);
			return;
		}
#elif defined(CONFIG_UART_IPC_RX)
		frame = (ipc_frame_t *)k_fifo_get(&rx_fifo, K_FOREVER);
		__ASSERT(frame != NULL, "RX frame is NULL");
		handle_rx_frame(frame);
#elif defined(CONFIG_UART_IPC_TX)
		frame = (ipc_frame_t *)k_fifo_get(&tx_fifo, K_FOREVER);
		__ASSERT(frame != NULL, "TX frame is NULL");
		handle_tx_frame(frame);
#endif /* CONFIG_UART_IPC_FULL */
	}
}

#if defined(CONFIG_UART_IPC_TX) || defined(CONFIG_UART_IPC_FULL)
int ipc_send_frame(ipc_frame_t *frame)
{
	int ret = -EINVAL;

	if (frame != NULL) {
		queue_tx_frame(frame);
		ret = 0;
	}

	return ret;
}

int ipc_send_data(const ipc_data_t *data)
{
	int ret;
	ipc_frame_t *frame;
	uint16_t size;
	
	/* checks */
	if (data == NULL) {
		return -EINVAL;
	}

	if (data->size > IPC_MAX_DATA_SIZE) {
		LOG_WRN("IPC data size too big = %d, forcing to %u",
			data->size, IPC_MAX_DATA_SIZE);
		size = IPC_MAX_DATA_SIZE;
	} else {
		size = data->size;
	}

	/* allocate */
	ret = alloc_frame(&frame);
	if (ret == 0) {
		/* copy data */
		memcpy(frame->data.buf, data->buf, size);
		frame->data.size = size;

		/* queue frame */
		queue_tx_frame(frame);
	}

	return ret;
}

int ipc_allocate_frame(ipc_frame_t **frame)
{
	return alloc_frame(frame);
}

#endif /* CONFIG_UART_IPC_TX */

#endif /* CONFIG_IPC_UART */
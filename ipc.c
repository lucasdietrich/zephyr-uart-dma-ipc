#include "ipc.h"

#include <kernel.h>

#include <device.h>
#include <devicetree.h>
#include <drivers/uart.h>

#if defined(CONFIG_UART_IPC)

#include <logging/log.h>
LOG_MODULE_REGISTER(ipc, LOG_LEVEL_DBG);

// TODO monitor usage using CONFIG_MEM_SLAB_TRACE_MAX_UTILIZATION

/**
 * @brief Enables logs in ISR context, caution: may unpredictably behavior
 */
#define LOGS_IN_ISR 0

// config
#define IPC_UART_RX_TIMEOUT_MS 1U
#define CONFIG_IPC_MEMSLAB_COUNT 2

// drivers
#define IPC_UART_NODE DT_ALIAS(ipc_uart)
#define CRC_NODE DT_NODELABEL(crc1)

static const struct device *uart_dev = DEVICE_DT_GET(IPC_UART_NODE);

// SHARED
static K_MEM_SLAB_DEFINE(frames_slab, IPC_FRAME_SIZE, CONFIG_IPC_MEMSLAB_COUNT, 4);


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
#if defined(CONFIG_UART_IPC_RX)
static uint8_t double_buffer[2][IPC_FRAME_SIZE];
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
#if defined(CONFIG_UART_IPC_TX)
static K_SEM_DEFINE(tx_sem, 1, 1);
K_FIFO_DEFINE(tx_fifo);

static uint32_t tx_seq = 0U;

static inline void queue_tx_frame(ipc_frame_t *frame)
{
	__ASSERT(frame != NULL, "frame is NULL");

	k_fifo_put(&tx_fifo, frame);
}
#endif /* UART_IPC_RX */

#if LOGS_IN_ISR
// convert uart_event to string
static const char *uart_event_to_str(enum uart_event_type type)
{
	switch (type) {
	case UART_TX_DONE:
		return "UART_TX_DONE";
	case UART_TX_ABORTED:
		return "UART_TX_ABORTED";
	case UART_RX_RDY:
		return "UART_RX_RDY";
	case UART_RX_BUF_REQUEST:
		return "UART_RX_BUF_REQUEST";
	case UART_RX_BUF_RELEASED:
		return "UART_RX_BUF_RELEASED";
	case UART_RX_DISABLED:
		return "UART_RX_DISABLED";
	case UART_RX_STOPPED:
		return "UART_RX_STOPPED";
	default:
		return "<UNKNOWN>";
	}
}
#endif

#if defined(CONFIG_UART_IPC_RX)
int ipc_attach_rx_msgq(struct k_msgq *msgq)
{
	rx_mxgq = msgq;

	return 0;
}

#if LOGS_IN_ISR
// convert enum parsing_state to string
static const char *parsing_state_to_str(parsing_state_t state)
{
	switch (state) {
	case PARSING_CATCH_FRAME:
		return "PARSING_CATCH_FRAME";
	case PARSING_FRAME_START_DELIMITER:
		return "PARSING_FRAME_START_DELIMITER";
	case PARSING_FRAME_DATA:
		return "PARSING_FRAME_DATA";
	default:
		return "<UNKNOWN>";
	}
}
#endif

/**
 * @brief Copy data from the buffer to the context frame
 * 
 * @param data ²
 * @param len 
 */
static inline void copy(uint8_t *data, size_t len)
{
	memcpy(ctx.raw + ctx.filling, data, len);
	ctx.filling += len;
}

static inline void copy_byte(const char chr)
{
	ctx.raw[ctx.filling++] = chr;
}

static void reset_ctx(void)
{
	ctx.frame = NULL;
	ctx.state = PARSING_CATCH_FRAME;
	ctx.filling = 0U;
}

/**
 * @brief This function is called from an ISR, so don't do any logging in it, 
 *  or except in particular cases
 * 
 * @param dev 
 * @param evt 
 * @param user_data 
 */
static void handle_received_chunk(const uint8_t *data, size_t size)
{
	int ret;

	while (size > 0) {
#if LOGS_IN_ISR
		LOG_DBG("%s", parsing_state_to_str(ctx.state));
#endif
		
		switch (ctx.state) {
		case PARSING_CATCH_FRAME:
		{
			/* looking for the first byte of the start frame delimiter */
			uint8_t *const p = (uint8_t *)
				memchr(data, IPC_START_FRAME_DELIMITER_BYTE, size);
			if (p != NULL) {
				/* if start frame delimiter is not at the very
				 * beginning of the chunk, we discard first bytes
				 */
				if (p != data) {
#if LOGS_IN_ISR
					LOG_WRN("Discarded %u B",
						(size_t)(p - data));
#endif
					/* adjust frame beginning */
					size -= (p - data);
					data = p;
				}

				/* try allocate a frame buffer */
				ret = alloc_frame(&ctx.frame);
				if (ret != 0) {
#if LOGS_IN_ISR
					LOG_ERR("Failed to allocate buf %d", ret);
#endif
					goto discard;
				}

				/* prepare context for the next state */
				ctx.remaining =
					IPC_START_FRAME_DELIMITER_SIZE - 1U;
				ctx.state = PARSING_FRAME_START_DELIMITER;
				copy_byte(*data);
				data++;
				size--;
			} else {
				/* not found */
				goto discard;
			}
			break;
		}
		case PARSING_FRAME_START_DELIMITER:
		{
			while (ctx.remaining > 0U && size > 0U) {
				if (data[0] == IPC_START_FRAME_DELIMITER_BYTE) {
					copy_byte(*data);
					ctx.remaining--;
					data++;
					size--;

				} else {
					goto discard;
				}
			}

			if (ctx.remaining == 0U) {
				ctx.state = PARSING_FRAME_DATA;
				ctx.remaining = IPC_FRAME_SIZE
					- IPC_START_FRAME_DELIMITER_SIZE;
			}
			break;
		}
		case PARSING_FRAME_DATA:
		{
			while (ctx.remaining > 0U && size > 0U) {
				ctx.remaining--;
				copy_byte(*data);
				data++;
				size--;
			}

			if (ctx.remaining == 0U) {
				__ASSERT(ctx.filling == IPC_FRAME_SIZE,
					 "Invalid frame size");

				__ASSERT(ctx.frame != NULL, "frame is NULL");

#if LOGS_IN_ISR
				LOG_DBG("====== FRAME COMPLETE ! seq = %x ======", 
					ctx.frame->seq);
#endif
				/* Pass the detected frame to the processing thread by
				* the FIFO
				* Note: As queuing a buffer to a FIFO requires 
				*  to write an address at the beginning, the start
				*  frame delimiter will be overwritten.
				*  However we don't need it anymore as it has been already
				*  checked above.
				*/				
				k_fifo_put(&rx_fifo, ctx.frame);

				/* reset context for next frame */
				reset_ctx();
			}
			break;
		}
		}
	}
	return;

discard:
#if LOGS_IN_ISR
	LOG_WRN("\t %s : Discarding %u B from state ",
		log_strdup(parsing_state_to_str(ctx.state)), size);
#endif

	free_frame(&ctx.frame);
	reset_ctx();
	return;
}

#endif

static void uart_callback(const struct device *dev,
			  struct uart_event *evt,
			  void *user_data)
{
#if LOGS_IN_ISR
	LOG_DBG("%s", uart_event_to_str(evt->type));
#endif 

	switch (evt->type) {
#if defined(CONFIG_UART_IPC_TX)
	case UART_TX_DONE:
		free_frame((ipc_frame_t **)&evt->data);
		k_sem_give(&tx_sem);
		break;
	case UART_TX_ABORTED:
		break;
#endif /* CONFIG_UART_IPC_TX */

#if defined(CONFIG_UART_IPC_RX)
	case UART_RX_RDY:
	{
#if LOGS_IN_ISR
		LOG_DBG("buf %x + %x : %u", (uint32_t) evt->data.rx.buf,
			evt->data.rx.offset, evt->data.rx.len);
		LOG_HEXDUMP_DBG(evt->data.rx.buf + evt->data.rx.offset,
				evt->data.rx.len, "RAW");
#endif

		handle_received_chunk(evt->data.rx.buf + evt->data.rx.offset,
				      evt->data.rx.len);
		break;
	}
	case UART_RX_BUF_REQUEST:
	{
		uart_rx_buf_rsp(dev, next_buffer, IPC_FRAME_SIZE);
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

K_THREAD_DEFINE(ipc_thread_id, 0x400, ipc_thread,
		NULL, NULL, NULL, K_PRIO_PREEMPT(8), 0, 0);

struct events {
#if defined(CONFIG_UART_IPC_RX)
	struct k_poll_event rx_ev;
#endif /* CONFIG_UART_IPC_RX */
#if defined(CONFIG_UART_IPC_TX)
	struct k_poll_event tx_ev;
#endif /* CONFIG_UART_IPC_TX */
};

static union {
	struct {
#if defined(CONFIG_UART_IPC_RX)
		struct k_poll_event rx_ev;
#endif /* CONFIG_UART_IPC_RX */
#if defined(CONFIG_UART_IPC_TX)
		struct k_poll_event tx_ev;
#endif /* CONFIG_UART_IPC_TX */
	};

	struct k_poll_event array[sizeof(struct events) / sizeof(struct k_poll_event)];
} events = {
#if defined(CONFIG_UART_IPC_RX)
	.rx_ev = K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_FIFO_DATA_AVAILABLE,
					  K_POLL_MODE_NOTIFY_ONLY,
					  &rx_fifo),
#endif /* CONFIG_UART_IPC_RX */
#if defined(CONFIG_UART_IPC_TX)
	.tx_ev = K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_FIFO_DATA_AVAILABLE,
					  K_POLL_MODE_NOTIFY_ONLY,
					  &tx_fifo),
#endif /* CONFIG_UART_IPC_TX */
};

#if defined(CONFIG_UART_IPC_RX)
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

#if defined(CONFIG_UART_IPC_TX)
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

#if defined(CONFIG_UART_IPC_RX)
	ret = uart_rx_enable(uart_dev,
			     double_buffer[0],
			     IPC_FRAME_SIZE,
			     IPC_UART_RX_TIMEOUT_MS);
	if (ret != 0) {
		LOG_ERR("uart_rx_enable() failed %d", ret);
		return;
	}
#endif /* CONFIG_UART_IPC_RX */

	for (;;) {
		ret = k_poll(events.array, ARRAY_SIZE(events.array), K_FOREVER);
		if (ret >= 0) {
#if defined(CONFIG_UART_IPC_RX)
			if (events.rx_ev.state == K_POLL_STATE_FIFO_DATA_AVAILABLE) {
				frame = (ipc_frame_t *)k_fifo_get(&rx_fifo, K_NO_WAIT);
				__ASSERT(frame != NULL, "RX frame is NULL");
				handle_rx_frame(frame);
			}
			events.rx_ev.state = K_POLL_STATE_NOT_READY;
#endif /* CONFIG_UART_IPC_RX */
#if defined(CONFIG_UART_IPC_TX)
			if (events.tx_ev.state == K_POLL_STATE_FIFO_DATA_AVAILABLE) {
				frame = (ipc_frame_t *)k_fifo_get(&tx_fifo, K_NO_WAIT);
				__ASSERT(frame != NULL, "TX frame is NULL");
				handle_tx_frame(frame);
			}
			events.tx_ev.state = K_POLL_STATE_NOT_READY;
#endif /* CONFIG_UART_IPC_TX */
		} else {
			LOG_ERR("k_poll() failed %d", ret);
			return;
		}
	}
}

#if defined(CONFIG_UART_IPC_TX)
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
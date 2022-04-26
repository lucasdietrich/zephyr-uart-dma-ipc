#include "ipc.h"

#include <zephyr.h>
#include <kernel.h>

#include <sys/crc.h>

#include <device.h>
#include <devicetree.h>
#include <drivers/uart.h>

#include "../utils/crc32_ieee.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(ipc_tx, LOG_LEVEL_DBG);

// config
#define IPC_UART_TX_TIMEOUT_MS 50U
#define CONFIG_IPC_MEMSLAB_COUNT 2

// drivers
#define IPC_UART_NODE DT_ALIAS(ipc_uart)

static const struct device *uart_dev = DEVICE_DT_GET(IPC_UART_NODE);

// internal
#define IPC_MEMSLAB_COUNT (CONFIG_IPC_MEMSLAB_COUNT)
K_MEM_SLAB_DEFINE(tx_frame_slab, IPC_FRAME_SIZE, IPC_MEMSLAB_COUNT, 4U);


static K_SEM_DEFINE(tx_finish_sem, 1, 1);

void tx_thread(void *_a, void *_b, void *_c);

static void wrap_ipc_frame(ipc_frame_t *frame, uint32_t sequence_number)
{
	frame->start_delimiter = IPC_START_FRAME_DELIMITER;
	frame->end_delimiter = IPC_END_FRAME_DELIMITER;

	/* set sequence number */
	frame->seq = sequence_number;

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

static void ipc_log_frame(const ipc_frame_t *frame)
{
	LOG_INF("IPC TX frame: %u B, seq = %x, data size = %u, sfd = %x, efd = %x crc32=%x",
		IPC_FRAME_SIZE, frame->seq, frame->data.size, frame->start_delimiter,
		frame->end_delimiter, frame->crc32);
	LOG_HEXDUMP_DBG(frame->data.buf, frame->data.size, "IPC frame data");
}
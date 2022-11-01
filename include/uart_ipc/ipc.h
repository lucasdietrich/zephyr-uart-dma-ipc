/**
 * @file ipc.h
 * @author Dietrich Lucas (ld.adecy@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-19
 * 
 * @copyright Copyright (c) 2022
 * 
 * SFD : Start Frame Delimiter
 * EFD : End Frame Delimiter
 * 
 */

#ifndef _IPC_H_	
#define _IPC_H_

#include <zephyr/kernel.h>

#include "ipc_frame.h"

struct ipc_stats {
#if defined(CONFIG_UART_IPC_RX) || defined(CONFIG_UART_IPC_FULL)
	struct {
		uint32_t bytes;
		uint32_t frames;
		uint32_t discarded_bytes; /* bytes discarded due to frame not recognized */
		uint32_t malformed; /* malformed frame (wrong length) */
		uint32_t crc_errors;
		uint32_t seq_gap; /* sequence number gap (due to lost frame or RX outage) */
		uint32_t frames_lost;
		uint32_t seq_reset; /* sequence number reset (due to peer reset for example) */
		uint32_t dropped_frames; /* frame dropped due to RX overrun or RX msgq not set */
		uint32_t unsupported_ver;
	} rx;
#endif
#if defined(CONFIG_UART_IPC_TX) || defined(CONFIG_UART_IPC_FULL)
	struct {
		uint32_t bytes;
		uint32_t frames;
		uint32_t retries;
		uint32_t errors;
	} tx;
#endif
	struct {
		uint32_t rx;
		uint32_t tx;
	} ping;
	
	struct {
		uint32_t mem_alloc_fail; /* allocation errors */
	} misc;
};

/**
 * @brief Start the IPC thread
 */
void ipc_thread_start(void);

/**
 * @brief Attach msg queue to retrieve RX messages
 * 
 * @param msgq 
 * @return int 
 */
int ipc_attach_rx_msgq(struct k_msgq *msgq);

/**
 * @brief Send data over IPC UART.
 * 
 * @retval -EINVAL  Invalid argument (data NULL or size > 255).
 * @retval 0	    If successful, negative errno code otherwise.
 */
int ipc_send_data(const ipc_data_t *data);

/**
 * @brief Allocate frame for IPC communication.
 * 
 * @param frame 
 * @return int 
 */
int ipc_allocate_frame(ipc_frame_t **frame);

/**
 * @brief Send the allocated frame.
 * 
 * @param frame 
 * @return int 
 */
int ipc_send_frame(ipc_frame_t *frame);

/**
 * @brief Free buffer for a frame that was not sent.
 * 
 * @param frame 
 */
void ipc_free_frame(ipc_frame_t **frame);

/**
 * @brief Retrieve the stats of the IPC module.
 * 
 * @param stats 
 */
void ipc_stats_get(struct ipc_stats *stats);

/**
 * @brief Reset the stats of the IPC module.
 */
void ipc_stats_reset(void);

typedef enum {
	IPC_LL_FRAME_RECEIVED = 1,
	IPC_DATA_FRAME_RECEIVED,
	IPC_PING_FRAME_RECEIVED,

	IPC_LL_FRAME_SENT,
} ipc_event_t;

typedef void (*ipc_event_handler_t)(ipc_event_t event,
				    void *data);

/**
 * @brief Allow the user to register an event handler for some events.
 * 
 * @param handler 
 * @param user_data 
 */
void ipc_register_event_callback(ipc_event_handler_t handler,
				 void *user_data);

#endif /* _IPC_H_ */
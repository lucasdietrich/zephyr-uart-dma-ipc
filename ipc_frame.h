#ifndef _IPC_FRAME_H_	
#define _IPC_FRAME_H_

#include <stdint.h>

#define IPC_MAX_DATA_SIZE 0x100U

#define FILL_UINT32_WITH_BYTE(byte) ((uint32_t) (((byte) & 0xFFU) \
	| (((byte) & 0xFFU) << 8U) \
	| (((byte) & 0xFFU) << 16U) \
	| (((byte) & 0xFFU) << 24U)))

#define IPC_START_FRAME_DELIMITER_BYTE 0xAAU
#define IPC_START_FRAME_DELIMITER FILL_UINT32_WITH_BYTE(IPC_START_FRAME_DELIMITER_BYTE)
#define IPC_START_FRAME_DELIMITER_SIZE sizeof(IPC_START_FRAME_DELIMITER)
 
#define IPC_PING_BYTE 0xFFU
#define IPC_PING FILL_UINT32_WITH_BYTE(IPC_PING_BYTE)

#define IPC_FRAME_SIZE sizeof(ipc_frame_t)

typedef struct {
	uint16_t size;
	uint8_t buf[IPC_MAX_DATA_SIZE];
} __attribute__((packed)) ipc_data_t;

enum {
	IPC_FRAME_TYPE_DATA = 0,
	IPC_FRAME_TYPE_PING,
};

typedef struct {
	/**
	 * @brief Start frame delimiter, 0xAAAAAAAAU
	 */
	uint32_t start_delimiter;

	/**
	 * @brief Sequence number of the frame
	 */
	uint32_t seq;

	/**
	 * @brief Version/type of the frame
	 */
	uint16_t ver;

	/**
	 * @brief Size and data in the frame
	 */
	ipc_data_t data;

	/**
	 * @brief CRC32 checksum
	 */
	uint32_t crc32;
} __attribute__((packed)) ipc_frame_t;

#endif /* _IPC_FRAME_H_ */
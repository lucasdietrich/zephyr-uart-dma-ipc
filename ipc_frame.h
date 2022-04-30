#ifndef _IPC_FRAME_H_	
#define _IPC_FRAME_H_

#include <stdint.h>

#define IPC_VERSION 2

#define IPC_MAX_DATA_SIZE 0x100U

#define MONITOR_EXEC 1

/* reason: CRC32 calculation */
#if IPC_MAX_DATA_SIZE & 0b11 != 0
#	error "IPC_MAX_DATA_SIZE must be a multiple of 4"
#endif

#define FILL_UINT32_WITH_BYTE(byte) ((uint32_t) (((byte) & 0xFFU) \
	| (((byte) & 0xFFU) << 8U) \
	| (((byte) & 0xFFU) << 16U) \
	| (((byte) & 0xFFU) << 24U)))

#define IPC_START_FRAME_DELIMITER_BYTE 0xAAU
#define IPC_START_FRAME_DELIMITER FILL_UINT32_WITH_BYTE(IPC_START_FRAME_DELIMITER_BYTE)
#define IPC_START_FRAME_DELIMITER_SIZE sizeof(IPC_START_FRAME_DELIMITER)

#define IPC_FRAME_SIZE sizeof(ipc_frame_t)

typedef struct {
	uint32_t size;
	uint8_t buf[IPC_MAX_DATA_SIZE];
} __attribute__((packed)) ipc_data_t;

typedef struct {
	uint32_t start_delimiter;
	uint32_t seq;
	ipc_data_t data;
	uint32_t crc32;
} __attribute__((packed)) ipc_frame_t;

#endif /* _IPC_FRAME_H_ */
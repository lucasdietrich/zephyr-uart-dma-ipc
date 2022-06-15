
if(CONFIG_UART_IPC)

	message("HELLO FROM UART_IPC CMakeLists.txt")

	zephyr_library()

	set(UART_IPC_PATH ${ZEPHYR_CURRENT_MODULE_DIR})

	message(${ZEPHYR_CURRENT_MODULE_DIR})

	zephyr_include_directories(${UART_IPC_PATH}/include)
	FILE(GLOB_RECURSE sources ${ZEPHYR_CURRENT_MODULE_DIR}/src/ipc.c)

	zephyr_library_sources(${sources})

endif()
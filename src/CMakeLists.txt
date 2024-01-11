cmake_minimum_required(VERSION 3.20)

set(FREERTOS_KERNEL_PATH ${PROJECT_SOURCE_DIR}/lib/FreeRTOS/)
add_library(freertos_config INTERFACE)

target_include_directories(freertos_config INTERFACE ${PROJECT_SOURCE_DIR})

set(FREERTOS_HEAP "4" CACHE STRING "" FORCE)
set(FREERTOS_PORT "GCC_ARM_CM3" CACHE STRING "" FORCE)
add_subdirectory(${FREERTOS_KERNEL_PATH} FreeRTOS-Kernel)


add_executable(Lidar Lidar.cpp opencm3.c uart_lib.h)
add_dependencies(Lidar libopencm3)
target_link_libraries(Lidar ${LIBOPENCM3_BLUEPILL_LIBRARIES} freertos_kernel freertos_config)

add_bin_from_elf(Lidar.bin Lidar)
add_stlink_upload_target(Lidar.bin)
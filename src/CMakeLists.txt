cmake_minimum_required(VERSION 3.20)

add_executable(Lidar Lidar.cpp)
add_dependencies(Lidar libopencm3)

set(FREERTOS_CONFIG_FILE_DIRECTORY ${PROJECT_SOURCE_DIR}/)
set(FREERTOS_PORT GCC_ARM_CM3 CACHE STRING "")
set(FREERTOS_HEAP 4 CACHE STRING "")

add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE 
    ${PROJECT_SOURCE_DIR}/lib/FreeRTOS/include
    ${PROJECT_SOURCE_DIR}
) 

target_compile_definitions(freertos_config INTERFACE projCOVERAGE_TEST=0)

add_subdirectory(${PROJECT_SOURCE_DIR}/lib/FreeRTOS ${CMAKE_BINARY_DIR}/FreeRTOS)
target_link_libraries(Lidar ${LIBOPENCM3_BLUEPILL_LIBRARIES} freertos_kernel freertos_config)

add_bin_from_elf(Lidar.bin Lidar)
add_stlink_upload_target(Lidar.bin)
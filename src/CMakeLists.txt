cmake_minimum_required(VERSION 3.20)

add_executable(Lidar 
  Lidar.cpp
  ${PROJECT_SOURCE_DIR}/rtos/heap_4.c
  ${PROJECT_SOURCE_DIR}/rtos/list.c
  ${PROJECT_SOURCE_DIR}/rtos/port.c
  ${PROJECT_SOURCE_DIR}/rtos/tasks.c
  ${PROJECT_SOURCE_DIR}/rtos/opencm3.c
)

target_include_directories(Lidar PUBLIC ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/rtos) 

add_dependencies(Lidar libopencm3)

target_link_libraries(Lidar ${LIBOPENCM3_BLUEPILL_LIBRARIES})

add_bin_from_elf(Lidar.bin Lidar)
add_stlink_upload_target(Lidar.bin)
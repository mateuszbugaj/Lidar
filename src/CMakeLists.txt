cmake_minimum_required(VERSION 3.20)

add_executable(Lidar Lidar.cpp)
add_dependencies(Lidar libopencm3)
target_link_libraries(Lidar ${LIBOPENCM3_BLUEPILL_LIBRARIES})

add_bin_from_elf(Lidar.bin Lidar)
add_stlink_upload_target(Lidar.bin)
cmake_minimum_required(VERSION 3.20)

add_library(VL53L0X STATIC VL53L0X.c VL53L0X.h)
target_include_directories(VL53L0X PUBLIC ${PROJECT_SOURCE_DIR}/src)
add_dependencies(VL53L0X libopencm3)
target_link_libraries(VL53L0X ${LIBOPENCM3_BLUEPILL_LIBRARIES})
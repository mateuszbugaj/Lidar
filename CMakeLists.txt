cmake_minimum_required(VERSION 3.20)

project(
  Lidar
  DESCRIPTION "Lidar project made with STM32F103, VL53L0X and FreeRTOS"
  LANGUAGES C CXX)

message("C compiler: " ${CMAKE_C_COMPILER} ", C++ compiler: " ${CMAKE_CXX_COMPILER})
include(${CMAKE_CURRENT_LIST_DIR}/cmake/blue-pill.cmake)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src)
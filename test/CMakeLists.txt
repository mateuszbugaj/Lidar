cmake_minimum_required(VERSION 3.20)

project(
  LidarTest
  DESCRIPTION "Test for lidar project"
  LANGUAGES C CXX)

message("C compiler: " ${CMAKE_C_COMPILER} ", C++ compiler: " ${CMAKE_CXX_COMPILER})
add_subdirectory(googletest)
enable_testing()

add_executable(LidarTest ControlerTest.cpp)
target_include_directories(LidarTest PUBLIC ../src/)
target_link_libraries(LidarTest gtest_main)
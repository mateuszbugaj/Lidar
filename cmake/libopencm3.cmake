if(TARGET libopencm3)
  return()
endif()

# Make sure that git submodule is initialized and updated
if (NOT EXISTS "/home/mat/Lidar/lib/libopencm3/Makefile")
  message(FATAL_ERROR "libopencm3 submodule not found. Initialize with 'git submodule update --init' in the source directory")
endif()

# Add a custom target to compile libopencm3
set(LIBOPENCM3_DIR /home/mat/Lidar/lib/libopencm3)
add_custom_target(
  libopencm3
  ALL
  make ${LIBOPENCM3_TARGETS}
  WORKING_DIRECTORY
  ${LIBOPENCM3_DIR}
)
include_directories(${LIBOPENCM3_DIR}/include)
link_directories(${LIBOPENCM3_DIR}/lib)

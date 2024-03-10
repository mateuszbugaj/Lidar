# Lidar

A 2 DOF manipulator with NEMA17 stepper motors driven by A4988 controllers.
There is a VL53L0X TOF distance sensor attached to the end-effector. </br>
Operations are coordinated with STM32F103 MCU running Free-RTOS and libopencm3 HAL library. 

Build with
```bash
cmake -S . -B build/ -DCMAKE_TOOLCHAIN_FILE=cmake/stm32-toolchain.cmake
cmake --build build/
```

Upload
```bash
make -C build/ Lidar.bin_upload
```

Run tests
```bash
cd test/
cmake --build build-test/
build-test/LidarTest
```
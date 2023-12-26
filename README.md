# Lidar

Build with
```bash
cmake -S . -B build/ -DCMAKE_TOOLCHAIN_FILE=cmake/stm32-toolchain.cmake
cmake --build build/
```

Run
```
make -C build/ Lidar.bin_upload
```
cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(mpu6050_read C CXX ASM)

pico_sdk_init()

add_executable(mpu6050_read
    main.c
)

target_link_libraries(mpu6050_read pico_stdlib hardware_i2c)

pico_enable_stdio_usb(mpu6050_read 1)
pico_enable_stdio_uart(mpu6050_read 0)

pico_add_extra_outputs(mpu6050_read)


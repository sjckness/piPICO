cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Default board: Pico 2 (RP2350). Override with -DPICO_BOARD=... if needed.
if (NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2w)
    set(PICO_PLATFORM rp2350)
endif()

project(mpu6050_read_easy C CXX ASM)

pico_sdk_init()

add_executable(mpu6050_read_easy
    main.c
)

target_link_libraries(mpu6050_read_easy m pico_stdlib hardware_i2c)

pico_enable_stdio_usb(mpu6050_read_easy 1)
pico_enable_stdio_uart(mpu6050_read_easy 0)

pico_add_extra_outputs(mpu6050_read_easy)

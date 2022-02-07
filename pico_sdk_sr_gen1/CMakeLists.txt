cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(test_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(pico_sdk_sr_gen_1
  pico_sdk_sr_gen_1.c
)

pico_enable_stdio_usb(pico_sdk_sr_gen_1 1)
pico_enable_stdio_uart(pico_sdk_sr_gen_1 0)

pico_add_extra_outputs(pico_sdk_sr_gen_1)

target_link_libraries(
    pico_sdk_sr_gen_1 
    pico_stdlib
    hardware_adc
    hardware_dma
    hardware_pio
    pico_multicore
)

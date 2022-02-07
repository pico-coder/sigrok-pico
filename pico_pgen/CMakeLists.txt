cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(test_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(pico_sdk_pgen
  pico_sdk_pgen.c
)

pico_enable_stdio_usb(pico_sdk_pgen 0)
pico_enable_stdio_uart(pico_sdk_pgen 1)

pico_add_extra_outputs(pico_sdk_pgen)

target_link_libraries(
    pico_sdk_pgen 
    pico_stdlib

)

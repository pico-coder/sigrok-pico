# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_sdk_sigrok C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
# DTR is not used for USB CDC connection, so we can set this to 1
# This will allow PulseView to connect independently of the DTR signal
add_definitions(-DPICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1)
pico_sdk_init()

add_executable(pico_sdk_sigrok
  pico_sdk_sigrok.c
  sr_device.c
)

pico_enable_stdio_usb(pico_sdk_sigrok 1)
pico_enable_stdio_uart(pico_sdk_sigrok 0)

pico_add_extra_outputs(pico_sdk_sigrok)

target_link_libraries(
    pico_sdk_sigrok 
    pico_stdlib
    hardware_adc
    hardware_dma
    hardware_pio
    hardware_sync
    pico_multicore
    pico_stdio_usb
)

# UART Bridge for Feather RP2350
# USB CDC <-> UART passthrough with NeoPixel status
# Bridges PC to Teensy 4.1 LPUART6 (KMBox command port)

cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD adafruit_feather_rp2350)
set(PICO_PLATFORM rp2350-arm-s)

# Custom board headers
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/../../RaspberryKMBox/boards)

# Pico SDK
if(NOT DEFINED PICO_SDK_PATH)
    if(DEFINED ENV{PICO_SDK_PATH})
        set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
    else()
        set(PICO_SDK_PATH $ENV{HOME}/.pico-sdk/sdk/2.2.0-fresh)
    endif()
endif()
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(uart_bridge C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(uart_bridge main.c)

target_include_directories(uart_bridge PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Generate ws2812.pio header from SDK
pico_generate_pio_header(uart_bridge
    ${PICO_SDK_PATH}/src/rp2_common/pico_status_led/ws2812.pio
)

target_link_libraries(uart_bridge
    pico_stdlib
    hardware_uart
    hardware_pio
    hardware_dma
    hardware_clocks
)

# USB CDC via pico_stdlib stdio (handles all TinyUSB descriptors)
pico_enable_stdio_usb(uart_bridge 1)
pico_enable_stdio_uart(uart_bridge 0)

pico_add_extra_outputs(uart_bridge)

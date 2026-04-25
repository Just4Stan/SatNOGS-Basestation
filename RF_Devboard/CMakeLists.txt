cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(satnogs_rx_fw C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(satnogs_rx_fw
    src/main.c
    src/leds.c

    # USB protocol
    src/usb_proto.c
    src/usb_cobs.c
    src/crc16.c
    src/cc1200_profile.c

    # CC1200 driver
    CC1200_driver/cc1200.c
    CC1200_driver/cc1200_hal.c
)

target_include_directories(satnogs_rx_fw PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/CC1200_driver
)

pico_enable_stdio_usb(satnogs_rx_fw 1)
pico_enable_stdio_uart(satnogs_rx_fw 0)

target_link_libraries(satnogs_rx_fw
    pico_stdlib
    hardware_gpio
    hardware_adc
    hardware_spi
    hardware_clocks
)

pico_add_extra_outputs(satnogs_rx_fw)

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rfm69_addressed C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Common RFM69 library
add_library(rfm69_lib STATIC
    common/rfm69.c
)

target_include_directories(rfm69_lib PUBLIC common)

# Sender executable
add_executable(rfm69_sender
    sender/main_sender.c
)

target_link_libraries(rfm69_sender
    pico_stdlib
    hardware_spi
    hardware_gpio
    rfm69_lib
)

pico_enable_stdio_usb(rfm69_sender 1)
pico_enable_stdio_uart(rfm69_sender 0)
pico_add_extra_outputs(rfm69_sender)

# Receiver executable
add_executable(rfm69_receiver
    receiver/main_receiver.c
)

target_link_libraries(rfm69_receiver
    pico_stdlib
    hardware_spi
    hardware_gpio
    rfm69_lib
)

pico_enable_stdio_usb(rfm69_receiver 1)
pico_enable_stdio_uart(rfm69_receiver 0)
pico_add_extra_outputs(rfm69_receiver)
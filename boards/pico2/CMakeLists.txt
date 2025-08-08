cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rfm69_addressed C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# --- Paths relative to boards/pico2 ---
set(BOARD_DIR    ${CMAKE_CURRENT_LIST_DIR})
set(COMMON_DIR   ${BOARD_DIR}/common)
set(SENDER_DIR   ${BOARD_DIR}/sender)
set(RECEIVER_DIR ${BOARD_DIR}/receiver)

# --- Common RFM69 library ---
add_library(rfm69_lib STATIC
    ${COMMON_DIR}/rfm69.c
)

# Headers for rfm69_lib live in 'boards/pico2/common'
target_include_directories(rfm69_lib PUBLIC
    ${COMMON_DIR}
)

# Propagate hw deps to users of the lib (PUBLIC so executables inherit)
target_link_libraries(rfm69_lib PUBLIC
    pico_stdlib
    hardware_spi
    hardware_gpio
)

# --- Sender executable ---
add_executable(rfm69_sender
    ${SENDER_DIR}/main_sender.c
)
target_link_libraries(rfm69_sender PRIVATE
    rfm69_lib
    pico_stdlib
)
pico_enable_stdio_usb(rfm69_sender 1)
pico_enable_stdio_uart(rfm69_sender 0)
pico_add_extra_outputs(rfm69_sender)

# --- Receiver executable ---
add_executable(rfm69_receiver
    ${RECEIVER_DIR}/main_receiver.c
)
target_link_libraries(rfm69_receiver PRIVATE
    rfm69_lib
    pico_stdlib
)
pico_enable_stdio_usb(rfm69_receiver 1)
pico_enable_stdio_uart(rfm69_receiver 0)
pico_add_extra_outputs(rfm69_receiver)

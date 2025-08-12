# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(piconet_node C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# --- Paths relative to current directory ---
set(CURRENT_DIR  ${CMAKE_CURRENT_LIST_DIR})
set(COMMON_DIR   ${CURRENT_DIR}/common)
set(SENDER_DIR   ${CURRENT_DIR}/sender)
set(RECEIVER_DIR ${CURRENT_DIR}/receiver)

# --- Common RFM69 library ---
add_library(rfm69_lib STATIC
    ${COMMON_DIR}/rfm69.c
)

# Headers for rfm69_lib live in 'common' directory
target_include_directories(rfm69_lib PUBLIC
    ${COMMON_DIR}
    ${CMAKE_CURRENT_LIST_DIR}
)

# Propagate hw deps to users of the lib (PUBLIC so executables inherit)
target_link_libraries(rfm69_lib PUBLIC
    pico_stdlib
    hardware_spi
    hardware_gpio
)

# --- Sender executable ---
add_executable(piconet_sender
    ${SENDER_DIR}/main_sender.c
)

pico_set_program_name(piconet_sender "piconet_sender")
pico_set_program_version(piconet_sender "0.1")

target_include_directories(piconet_sender PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(piconet_sender PRIVATE
    rfm69_lib
    pico_stdlib
)

pico_enable_stdio_usb(piconet_sender 1)
pico_enable_stdio_uart(piconet_sender 0)
pico_add_extra_outputs(piconet_sender)

# --- Receiver executable ---
add_executable(piconet_receiver
    ${RECEIVER_DIR}/main_receiver.c
)

pico_set_program_name(piconet_receiver "piconet_receiver")
pico_set_program_version(piconet_receiver "0.1")

target_include_directories(piconet_receiver PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(piconet_receiver PRIVATE
    rfm69_lib
    pico_stdlib
)

pico_enable_stdio_usb(piconet_receiver 1)
pico_enable_stdio_uart(piconet_receiver 0)
pico_add_extra_outputs(piconet_receiver)

# --- Simple test executable ---
add_executable(simple_test
    simple_test.c
)

pico_set_program_name(simple_test "simple_test")
pico_set_program_version(simple_test "0.1")

target_link_libraries(simple_test PRIVATE
    pico_stdlib
    hardware_gpio
)

pico_enable_stdio_usb(simple_test 1)
pico_enable_stdio_uart(simple_test 0)
pico_add_extra_outputs(simple_test)

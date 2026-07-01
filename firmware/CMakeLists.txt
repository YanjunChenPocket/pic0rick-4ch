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

# Board type: original Raspberry Pi Pico / RP2040.
# Defaults to pico if not specified.
if(NOT PICO_BOARD)
    set(PICO_BOARD pico CACHE STRING "Board type")
endif()

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pic0rick-tx-short-mux C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(pic0rick-tx-short-mux)

pico_set_program_name(pic0rick-tx-short-mux "pic0rick-tx-short-mux")
pico_set_program_version(pic0rick-tx-short-mux "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pic0rick-tx-short-mux 0)
pico_enable_stdio_usb(pic0rick-tx-short-mux 1)

pico_generate_pio_header(pic0rick-tx-short-mux ${CMAKE_CURRENT_LIST_DIR}/adc/adc.pio)
pico_generate_pio_header(pic0rick-tx-short-mux ${CMAKE_CURRENT_LIST_DIR}/max/max14866.pio)

target_sources(pic0rick-tx-short-mux PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/main.c
    ${CMAKE_CURRENT_LIST_DIR}/adc/adc.c
    ${CMAKE_CURRENT_LIST_DIR}/max/max14866.c
)

# Add the standard include files to the build
target_include_directories(pic0rick-tx-short-mux PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(pic0rick-tx-short-mux PRIVATE
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_clocks
    hardware_gpio
    hardware_irq
)

pico_add_extra_outputs(pic0rick-tx-short-mux)

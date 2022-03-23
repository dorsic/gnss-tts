cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{PICO_SDK_PATH}/tools/CMakeLists.txt)

project(countermodule C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(${PROJECT_NAME} 
    counter_module.cpp
    SCPI_Parser/Vrekrer_scpi_parser.cpp
)

pico_set_program_name(${PROJECT_NAME} "PicoDIV_CM2")
pico_set_program_version(${PROJECT_NAME} "2.1.0")

pico_generate_pio_header(${PROJECT_NAME}  ${CMAKE_CURRENT_LIST_DIR}/picoDIV.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/picoDIV_sync.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/picoPET_sp.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/uart.pio)

target_sources(${PROJECT_NAME} 
    PRIVATE counter_module.cpp
    SCPI_Parser/Vrekrer_scpi_parser.cpp
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    pico_stdio
    pico_multicore
    hardware_timer
    hardware_pll
    hardware_xosc
    hardware_pio
    hardware_spi
 )

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})

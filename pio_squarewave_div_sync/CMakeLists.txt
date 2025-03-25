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
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pio_squarewave_div_sync C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(pio_squarewave)

pico_generate_pio_header(pio_squarewave ${CMAKE_CURRENT_LIST_DIR}/squarewave.pio)
pico_generate_pio_header(pio_squarewave ${CMAKE_CURRENT_LIST_DIR}/squarewave_wrap.pio)
pico_generate_pio_header(pio_squarewave ${CMAKE_CURRENT_LIST_DIR}/squarewave_fast.pio)

target_sources(pio_squarewave PRIVATE squarewave.c)

target_link_libraries(pio_squarewave PRIVATE pico_stdlib hardware_pio)
pico_add_extra_outputs(pio_squarewave)

# add url via pico_set_program_url

file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/generated)
# generate .hex file and .pio.h file for the RP2040 datasheet (to make sure
# the datasheet always shows the output of the latest pioasm version)
add_custom_target(pio_squarewave_datasheet DEPENDS
		${CMAKE_CURRENT_LIST_DIR}/generated/squarewave.hex
		${CMAKE_CURRENT_LIST_DIR}/generated/squarewave.pio.h
		${CMAKE_CURRENT_LIST_DIR}/generated/squarewave_wrap.pio.h
		)
add_custom_command(OUTPUT ${CMAKE_CURRENT_LIST_DIR}/generated/squarewave.hex
        DEPENDS ${CMAKE_CURRENT_LIST_DIR}/squarewave.pio
        COMMAND pioasm -o hex ${CMAKE_CURRENT_LIST_DIR}/squarewave.pio ${CMAKE_CURRENT_LIST_DIR}/generated/squarewave.hex
        VERBATIM)
add_custom_command(OUTPUT ${CMAKE_CURRENT_LIST_DIR}/generated/squarewave.pio.h
        DEPENDS ${CMAKE_CURRENT_LIST_DIR}/squarewave.pio
        COMMAND pioasm ${CMAKE_CURRENT_LIST_DIR}/squarewave.pio ${CMAKE_CURRENT_LIST_DIR}/generated/squarewave.pio.h
        VERBATIM)
add_custom_command(OUTPUT ${CMAKE_CURRENT_LIST_DIR}/generated/squarewave_wrap.pio.h
        DEPENDS ${CMAKE_CURRENT_LIST_DIR}/squarewave_wrap.pio
        COMMAND pioasm ${CMAKE_CURRENT_LIST_DIR}/squarewave_wrap.pio ${CMAKE_CURRENT_LIST_DIR}/generated/squarewave_wrap.pio.h
        VERBATIM)
add_dependencies(pio_squarewave pio_squarewave_datasheet)

add_executable(pio_squarewave_div_sync)
pico_generate_pio_header(pio_squarewave_div_sync ${CMAKE_CURRENT_LIST_DIR}/squarewave.pio)
target_sources(pio_squarewave_div_sync PRIVATE
    squarewave_div_sync.c
    )
target_link_libraries(pio_squarewave_div_sync PRIVATE
    pico_stdlib hardware_pio
    )
pico_add_extra_outputs(pio_squarewave_div_sync)

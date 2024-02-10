cmake_minimum_required(VERSION 3.27.7)

project(quadcopter_dsp)

set(MCU "stm32f103c6t6" CACHE STRING "MCU target")

#This works as I expected
if(MCU STREQUAL "stm32f103c8t6")
    message("Selected MCU: stm32f103c8t6")

    # Set environment variables
    set(MCU_FAMILY STM32F1xx)
    set(MCU_MODEL STM32F103X8)
    set(STARTUP_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f103x8.s)
    set(LINKER_SCRIPT  ${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld)

elseif(MCU STREQUAL "stm32f103c6t6")
    message("Selected Default MCU: stm32f103c6t6")

    # Set environment variables
    set(MCU_FAMILY STM32F1xx)
    set(MCU_MODEL STM32F103X6)
    set(STARTUP_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f103x6.s)
    set(LINKER_SCRIPT  ${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C6Tx_FLASH.ld)

else()
    message("Bad MCU name")
endif()

# Headers
set(CUBEMX_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/hal
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/${MCU_FAMILY}/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include)

set(PROJECT_INCLUDE_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/dsp)

# Sources
file(GLOB_RECURSE STM32CUBEMX_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/hal/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/*.c)

file(GLOB_RECURSE PROJECT_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/drivers/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dsp/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/*.c)


# Add executable
add_executable()
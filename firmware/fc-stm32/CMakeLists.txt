cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME fc-stm32)

# Include toolchain file
include(gcc-arm-none-eabi.cmake)

# Include functions file
include(functions.cmake)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
    Core/Src/drivers/bmp280.c
    Core/Src/drivers/flash_memory.c
    Core/Src/drivers/ibus.c
    Core/Src/drivers/motors.c
    Core/Src/drivers/mpu6050.c
    Core/Src/drivers/nrf24l01.c
    Core/Src/drivers/w25q64.c

    Core/Src/dsp/angle_estimation.c
    Core/Src/dsp/filters.c
    Core/Src/dsp/pid.c
    Core/Src/dsp/stabilizer.c

    Core/Src/dma.c
    Core/Src/gpio.c
    Core/Src/i2c.c
    Core/Src/logger.c
    Core/Src/rc.c
    Core/Src/spi.c
    Core/Src/tim.c
    Core/Src/usart.c
    
    Core/Src/main.c

    Core/Src/syscalls.c
    Core/Src/sysmem.c

    Drivers/STM32F1xx/Src/stm32f1xx_hal_msp.c
    Drivers/STM32F1xx/Src/stm32f1xx_it.c
    Drivers/STM32F1xx/Src/system_stm32f1xx.c
    Drivers/STM32F1xx/startup_stm32f103xb.s
)

include_directories(
    Core/Inc
    Core/Inc/drivers
    Core/Inc/dsp
)

# Add STM32CubeMX generated sources
add_subdirectory(Drivers/STM32F1xx_HAL_Driver)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32f1xxHal
)

target_stlink_flash(${CMAKE_PROJECT_NAME})
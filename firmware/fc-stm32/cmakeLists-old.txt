cmake_minimum_required(VERSION 3.27.7)

project(quadcopter_dsp)

set(MCU "stm32f103c6t6" CACHE STRING "MCU target")

if(MCU STREQUAL "stm32f103c8t6")
    message("Selected MCU: stm32f103c8t6")

    # Set environment variables
    set(MCU_FAMILY STM32F1xx)
    set(MCU_MODEL STM32F103X8)
    set(STARTUP_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f103x8.s)
    set(LINKER_SCRIPT  ${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld)
    
    set(CPU_PARAM
            -mcpu=cortex-m3
            -mthumb #M-profile processors execute T32 (thumb) instructions
            #-mfpu=
            #-mfloat=
        )

elseif(MCU STREQUAL "stm32f103c6t6")
    message("Selected Default MCU: stm32f103c6t6")

    # Set environment variables
    set(MCU_FAMILY STM32F1xx)
    set(MCU_MODEL STM32F103X6)
    set(STARTUP_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f103x6.s)
    set(LINKER_SCRIPT  ${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C6Tx_FLASH.ld)

    set(CPU_PARAM
            -mcpu=cortex-m3
            -mthumb #M-profile processors execute T32 (thumb) instructions
            #-mfpu=
            #-mfloat=
        )

else()
    message("Bad MCU name")
endif()

set(EXECUTABLE ${CMAKE_PROJECT_NAME})
enable_language(C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

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

file(GLOB_RECURSE PROJECT_SOURCES FOLLOW_SYMLINKS
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/drivers/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dsp/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/*.c)


# Add executable
add_executable(${EXECUTABLE}
                ${STM32CUBEMX_SOURCES}
                ${PROJECT_SOURCES}
                ${STARTUP_SCRIPT}
                )

# Add required defines
target_compile_definitions(${EXECUTABLE} PRIVATE
                           ${MCU_MODEL}
                           USE_HAL_DRIVER)

# Add include dirs
target_include_directories(${EXECUTABLE} PRIVATE
                           ${CUBEMX_INCLUDE_DIRS}
                           ${PROJECT_INCLUDE_DIRS}
                          ) 


# # Target compiler options
# target_compile_options(${EXECUTABLE} PRIVATE
#                         ${CPU_PARAM}
#                         -Wall                    # enable compiler warnings
#                         -Wunused-parameter       # waring on unused param
#                         -Wextra                  # enable an additional set of flags
#                         $<$<COMPILE_LANGUAGE:CXX>:
#                         -Wuseless-cast
#                         -Wsuggest-override>
#                         -Og
#                         -g0)

# # Target linker options
# target_link_options(${EXECUTABLE} PRIVATE
#                         ${LINKER_SCRIPT}
#                         ${CPU_PARAM}
#                         - Wl, -Map={CMAKE_PROJECT_NAME}.map
#                         --specs= nosys.specs        # include dummy versions for exit()
#                         -Wl,--start-group
#                         -lc                         # central lib
#                         -lm                         # math lib
#                         -lnosys 
#                         -lstdc++
#                         -Wl,--end-group
#                         -Wl,--print-memory-usage)   # print memory usage

# Compiler and linker options
target_compile_options(${EXECUTABLE} PRIVATE
    ${CPU_PARAMETERS}
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
    $<$<COMPILE_LANGUAGE:CXX>:
        -Wno-volatile
        -Wold-style-cast
        -Wuseless-cast
        -Wsuggest-override>
    $<$<CONFIG:Debug>:-Og -g3 -ggdb>
    $<$<CONFIG:Release>:-Og -g0>)

target_link_options(${EXECUTABLE} PRIVATE
    -T${MCU_LINKER_SCRIPT}
    ${CPU_PARAMETERS}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    --specs=nosys.specs
    -Wl,--start-group
    -lc
    -lm
    -lstdc++
    -Wl,--end-group
    -Wl,--print-memory-usage)


add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE}>)
                    
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}>
    ${EXECUTABLE}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}>
    ${EXECUTABLE}.bin)
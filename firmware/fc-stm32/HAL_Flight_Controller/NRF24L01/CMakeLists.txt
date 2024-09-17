# Add nrf24l01 library
add_library(nrf24l01
    nrf24l01.c
)

# Add include directories
target_include_directories(nrf24l01
    PUBLIC
        ${CMAKE_SOURCE_DIR}/HAL_Flight_Controller/NRF24L01
)

# Include hal subdirectory
add_subdirectory(${CMAKE_SOURCE_DIR}/HAL_STM32/${MCU_FAMILY}_HAL_Driver)

# Add linked libraries
target_link_libraries(nrf24l01 ${MCU_FAMILY_LOWER}Hal)
cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME sensores_avionica_g4_v0)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)
add_subdirectory(third_party/WESensorsSDK)
add_subdirectory(third_party/minmea)
add_subdirectory(third_party/ST_LSM6DSV32X)
add_subdirectory(third_party/can_protocol)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    # Drivers propios
    Drivers/Sensors/vectorNav.c
    Drivers/Sensors/barometer.c
    Drivers/Sensors/gps.c
    Drivers/Sensors/imu.c
    Drivers/Sensors/imu2.c
    Drivers/Sensors/magnetometer.c
    Drivers/Timestamp/timestamp.c
    Drivers/Servo/servo.c
    Drivers/Can_driver/canTelemetry.c
    Drivers/Can_driver/canCommand.c
    Drivers/Can_driver/canNode.c
    Drivers/Can_driver/canBusOff.c
    Drivers/Watchdog/watchdog.c

    # Firmware de vuelo (App/). Para compilar una PoC, comenta este bloque
    # (todas las líneas App/) y descomenta UNA sola línea poc/.../main.c
    App/main.c
    App/sensorIrq.c
    App/sensorsI2c.c
    App/sensorsUart.c
    App/calibration.c

    # poc/wsen_pads/main.c
    # poc/VectorNav_VN100/main.c
    # poc/blinking_led/main.c
    # poc/ublox_gnss/main.c
    # poc/wsen_isds/main.c
    # poc/i2c_scan/main.c
    # poc/mlx90393/main.c
    # poc/lsm6dsv32x/main.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App
    Core/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32G4xx/Include
    Drivers/CMSIS/Include
    Drivers/Sensors
    Drivers/Timestamp
    Drivers/Servo
    Drivers/Can_driver
    Drivers/Watchdog
    # Add user defined include paths
)

# Link stm32cubemx for HAL includes and compiler paths
target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    WE_USE_FLOAT
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    wsensdk
    minmea
    lsm6dsv32x
    can_protocol
    # Add user defined libraries
)

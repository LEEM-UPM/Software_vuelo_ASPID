cmake_minimum_required(VERSION 3.22)

# STMicroelectronics' official platform-independent driver (PID) for the
# LSM6DSV32X, vendored from https://github.com/STMicroelectronics/lsm6dsv32x-pid
# (BSD-3-Clause, see LICENSE). Only the register-level layer is used --
# Core/Src/imu2.c provides the I2C glue and the board-level API.

project(lsm6dsv32x)
add_library(lsm6dsv32x INTERFACE)

target_include_directories(lsm6dsv32x INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(lsm6dsv32x INTERFACE
    lsm6dsv32x_reg.c
)

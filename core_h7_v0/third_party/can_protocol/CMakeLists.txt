cmake_minimum_required(VERSION 3.22)

project(can_protocol LANGUAGES C)

add_library(can_protocol INTERFACE)

target_include_directories(can_protocol INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(can_protocol INTERFACE c_std_11)

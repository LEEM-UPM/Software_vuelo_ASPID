cmake_minimum_required(VERSION 3.22)

project(minmea)
add_library(minmea INTERFACE)

target_include_directories(minmea INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(minmea INTERFACE
    minmea.c
)

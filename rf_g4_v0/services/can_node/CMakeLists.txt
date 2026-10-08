# services/can_node/CMakeLists.txt
#
# This board as a node of the common CAN1 bus (ICD in
# third_party/can_protocol). Sits on top of bsp_can1; no STM32 HAL here.

add_library(service_can_node STATIC)

target_sources(service_can_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/can_node.c
)

target_include_directories(service_can_node PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(service_can_node PUBLIC
    bsp_can1
    can_protocol
)

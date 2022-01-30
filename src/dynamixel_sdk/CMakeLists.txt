################################################################################
# CMake
################################################################################
# cmake_minimum_required(VERSION 3.0.0)
# project(dynamixel_sdk)

################################################################################
# Set variables
################################################################################

set(REPLACEMENTS ${CMAKE_CURRENT_SOURCE_DIR}/src/ros_replacements)

set(SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/src/packet_handler.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/protocol1_packet_handler.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/protocol2_packet_handler.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/group_sync_read.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/group_sync_write.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/group_bulk_read.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/group_bulk_write.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/port_handler.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/port_handler_linux.cpp
)
set(HEADERS
  ${CMAKE_CURRENT_LIST_DIR}/include/dynamixel_sdk/dynamixel_sdk.h
  ${CMAKE_CURRENT_LIST_DIR}/include/dynamixel_sdk/packet_handler.h
  ${CMAKE_CURRENT_LIST_DIR}/include/dynamixel_sdk/protocol1_packet_handler.h
  ${CMAKE_CURRENT_LIST_DIR}/include/dynamixel_sdk/protocol2_packet_handler.h
  ${CMAKE_CURRENT_LIST_DIR}/include/dynamixel_sdk/group_sync_read.h
  ${CMAKE_CURRENT_LIST_DIR}/include/dynamixel_sdk/group_sync_write.h
  ${CMAKE_CURRENT_LIST_DIR}/include/dynamixel_sdk/group_bulk_read.h
  ${CMAKE_CURRENT_LIST_DIR}/include/dynamixel_sdk/group_bulk_write.h
  ${CMAKE_CURRENT_LIST_DIR}/include/dynamixel_sdk/port_handler.h
  ${CMAKE_CURRENT_LIST_DIR}/include/dynamixel_sdk/port_handler_linux.h
)

################################################################################
# Build
################################################################################
add_library(dynamixel_sdk
  ${SOURCES}
  ${HEADERS}
  ${REPLACEMENTS}/ros_replacements/status_output.hpp
)

target_include_directories(dynamixel_sdk PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include ${REPLACEMENTS})
# add_dependencies(dynamixel_sdk ${${PROJECT_NAME}_EXPORTED_TARGETS})

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if (${ARCHITECTURE} MATCHES "arm")
    message(STATUS "wiringPi library is required - arm processor")
    target_link_libraries(dynamixel_sdk
        -lwiringPi
    )
else()
    message(STATUS "wiringPi library not required")
endif()
################################################################################
# Install
################################################################################
# install(TARGETS dynamixel_sdk
#   DESTINATION lib
# )
# install(FILES ${HEADERS} DESTINATION lib/include)

################################################################################
# Test
################################################################################

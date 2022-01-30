cmake_minimum_required(VERSION 3.0.0)
project(niryo_one_driver VERSION 0.1.0)

set(DYNAMIXEL_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamixel_sdk)
set(MCP_CAN_RPI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/mcp_can_rpi)
set(REPLACEMENTS ${CMAKE_CURRENT_SOURCE_DIR}/src/ros_replacements)

set(SOURCES_COMM
  ${CMAKE_CURRENT_LIST_DIR}/src/hw_comm/can_communication.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/hw_comm/dxl_communication.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/hw_comm/fake_communication.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/hw_comm/niryo_one_communication.cpp
)
set(SOURCES_DRIVER
  ${CMAKE_CURRENT_LIST_DIR}/src/hw_driver/dxl_driver.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/hw_driver/niryo_one_can_driver.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/hw_driver/xl320_driver.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/hw_driver/xl430_driver.cpp
)
set(SOURCES_UTILS
  ${CMAKE_CURRENT_LIST_DIR}/src/utils/change_hardware_version.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/utils/motor_offset_file_handler.cpp
)
set(SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/src/niryo_one_driver_node.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/niryo_one_hardware_interface.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/ros_interface.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/rpi_diagnostics.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/test_motors.cpp
)
set(HEADERS
  ${CMAKE_CURRENT_LIST_DIR}/include/niryo_one_driver/can_communication.h
  ${CMAKE_CURRENT_LIST_DIR}/include/niryo_one_driver/change_hardware_version.h
  ${CMAKE_CURRENT_LIST_DIR}/include/niryo_one_driver/communication_base.h
  ${CMAKE_CURRENT_LIST_DIR}/include/niryo_one_driver/dxl_communication.h
  ${CMAKE_CURRENT_LIST_DIR}/include/niryo_one_driver/dxl_driver.h
  ${CMAKE_CURRENT_LIST_DIR}/include/niryo_one_driver/dxl_motor_state.h
  ${CMAKE_CURRENT_LIST_DIR}/include/niryo_one_driver/fake_communication.h
  ${CMAKE_CURRENT_LIST_DIR}/include/niryo_one_driver/motor_offset_file_handler.h
  ${CMAKE_CURRENT_LIST_DIR}/include/niryo_one_driver/niryo_one_can_driver.h
  ${CMAKE_CURRENT_LIST_DIR}/include/niryo_one_driver/niryo_one_communication.h
  ${CMAKE_CURRENT_LIST_DIR}/include/niryo_one_driver/niryo_one_hardware_interface.h
  ${CMAKE_CURRENT_LIST_DIR}/include/niryo_one_driver/ros_interface.h
  ${CMAKE_CURRENT_LIST_DIR}/include/niryo_one_driver/rpi_diagnostics.h
  ${CMAKE_CURRENT_LIST_DIR}/include/niryo_one_driver/stepper_motor_state.h
  ${CMAKE_CURRENT_LIST_DIR}/include/niryo_one_driver/test_motors.h
  ${CMAKE_CURRENT_LIST_DIR}/include/niryo_one_driver/xl320_driver.h
  ${CMAKE_CURRENT_LIST_DIR}/include/niryo_one_driver/xl430_driver.h
)

add_library(niryo_one_driver
  ${SOURCES_COMM}
  ${SOURCES_DRIVER}
  ${SOURCES_UTILS}
  ${SOURCES}
  ${HEADERS}
  ${REPLACEMENTS}/ros_replacements/ros_time_repl.hpp
  ${REPLACEMENTS}/ros_replacements/status_output.hpp
  ${MCP_CAN_RPI_DIR}/include/mcp_can_rpi/mcp_can_rpi.h
  ${DYNAMIXEL_SDK_DIR}/include/dynamixel_sdk/dynamixel_sdk.h
)
target_include_directories(niryo_one_driver PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(niryo_one_driver PUBLIC ${DYNAMIXEL_SDK_DIR}/include ${MCP_CAN_RPI_DIR}/include ${REPLACEMENTS})

add_dependencies(niryo_one_driver dynamixel_sdk mcp_can_rpi)

#
# wiringPi should be installed only on a Raspberry Pi board
#

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if (${ARCHITECTURE} MATCHES "arm")
    message(STATUS "wiringPi library is required - arm processor")
    target_link_libraries(niryo_one_driver
        -lwiringPi
    )
else()
    message(STATUS "wiringPi library not required")
endif()

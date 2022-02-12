cmake_minimum_required(VERSION 3.0.0)
project(manus_node)

find_package(echolib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(manus REQUIRED)
find_package(Threads REQUIRED)

set(HEADERS
    ${CMAKE_CURRENT_LIST_DIR}/include/manus_node/manus_node.h
)
set(SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/manus_node.cpp
)

add_executable(manus_node ${SOURCES} ${HEADERS})
target_link_libraries(manus_node manus_interface change_led ros_replacements ${manus_LIBRARIES} ${echolib_LIBRARIES} ${OpenServo_LIBRARIES} ${YAML_CPP_LIBRARIES} Threads::Threads)
target_include_directories(manus_node PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include manus_interface change_led ros_replacements ${manus_INCLUDE_DIRS} ${echolib_INCLUDE_DIRS} ${OpenServo_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})
add_dependencies(manus_node manus_interface change_led)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if (${ARCHITECTURE} MATCHES "arm")
    message(STATUS "wiringPi library is required for manus_node (ARM processor)")
    target_link_libraries(mcp_can_rpi
        -lwiringPi
    )
else()
    message(STATUS "wiringPi library not required")
endif()

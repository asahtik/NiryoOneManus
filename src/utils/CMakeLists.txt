cmake_minimum_required(VERSION 3.0.0)
project(utils)

set(HEADERS 
    ${CMAKE_CURRENT_LIST_DIR}/include/utils/change_led.h
)
set(SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/change_led.cpp
)

add_library(change_led ${SOURCES} ${HEADERS})
target_include_directories(change_led PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if (${ARCHITECTURE} MATCHES "arm")
    message(STATUS "wiringPi library is required for utils (ARM processor)")
    target_link_libraries(mcp_can_rpi
        -lwiringPi
    )
else()
    message(STATUS "wiringPi library not required")
endif()
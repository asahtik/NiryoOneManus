cmake_minimum_required(VERSION 3.0.0)
project(ros_replacements VERSION 0.1.0)

set(SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/ros_time_repl.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/status_output.cpp
)
set(HEADERS
    ${CMAKE_CURRENT_LIST_DIR}/include/ros_replacements/ros_time_repl.h
    ${CMAKE_CURRENT_LIST_DIR}/include/ros_replacements/status_output.h
)

add_library(ros_replacements ${SOURCES} ${HEADERS})

target_include_directories(ros_replacements PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
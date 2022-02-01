cmake_minimum_required(VERSION 3.0.0)
project(manus_interface VERSION 0.1.0)

set(REPLACEMENTS ${CMAKE_CURRENT_SOURCE_DIR}/src/ros_replacements)
set(DRIVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/niryo_one_driver)
set(SOURCES 
    ${CMAKE_CURRENT_LIST_DIR}/src/manus_interface.cpp
)
set(HEADERS
    ${CMAKE_CURRENT_LIST_DIR}/include/manus_interface/manus_interface.h
)

add_library(manus_interface
    ${SOURCES}
    ${HEADERS}
    ${REPLACEMENTS}/ros_replacements/ros_time_repl.hpp
    ${REPLACEMENTS}/ros_replacements/status_output.hpp
    ${DRIVER_DIR}/include/niryo_one_driver/niryo_one_communication.h
)

target_link_libraries(manus_interface niryo_one_driver)

target_include_directories(manus_interface PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include ${REPLACEMENTS} ${DRIVER_DIR}/include niryo_one_driver)

add_dependencies(manus_interface niryo_one_driver)
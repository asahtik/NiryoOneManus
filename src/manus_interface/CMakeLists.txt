cmake_minimum_required(VERSION 3.0.0)
project(manus_interface VERSION 0.1.0)

find_package(yaml-cpp REQUIRED)

set(DRIVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/niryo_one_driver)
set(SOURCES 
    ${CMAKE_CURRENT_LIST_DIR}/src/manus_interface.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/parse_description.cpp
)
set(HEADERS
    ${CMAKE_CURRENT_LIST_DIR}/include/manus_interface/manus_interface.h
    ${CMAKE_CURRENT_LIST_DIR}/include/manus_interface/parse_description.h
)

add_library(manus_interface
    ${SOURCES}
    ${HEADERS}
)

target_link_libraries(manus_interface ros_replacements niryo_one_driver ${YAML_CPP_LIBRARIES})

target_include_directories(manus_interface PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include ros_replacements niryo_one_driver ${YAML_CPP_INCLUDE_DIRS})

add_dependencies(manus_interface niryo_one_driver)
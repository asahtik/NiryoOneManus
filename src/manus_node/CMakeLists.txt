cmake_minimum_required(VERSION 3.0.0)
project(manus_node)

find_package(echolib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(manus REQUIRED)

find_library(WPI_LIB NAMES wiringPi REQUIRED)
find_path(WPI_INCLUDE NAMES wiringPi.h REQUIRED)

set(HEADERS
    ${CMAKE_CURRENT_LIST_DIR}/include/manus_node/manus_node.h
)
set(SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/manus_node.cpp
)

add_executable(manus_node ${SOURCES} ${HEADERS})
target_link_libraries(manus_node manus_interface change_led ros_replacements ${manus_LIBRARIES} ${echolib_LIBRARIES} ${OpenServo_LIBRARIES} ${YAML_CPP_LIBRARIES} ${WPI_LIB})
target_include_directories(manus_node PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include manus_interface change_led ros_replacements ${manus_INCLUDE_DIRS} ${echolib_INCLUDE_DIRS} ${OpenServo_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS} ${WPI_INCLUDE})
add_dependencies(manus_node manus_interface change_led)

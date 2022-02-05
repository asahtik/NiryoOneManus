cmake_minimum_required(VERSION 3.0.0)
project(utils)

set(HEADERS 
    ${CMAKE_CURRENT_LIST_DIR}/include/utils/change_led.h
)
set(SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/change_led.cpp
)

find_library(WPI_LIB NAMES wiringPi REQUIRED)
find_path(WPI_INCLUDE NAMES wiringPi.h REQUIRED)

add_library(change_led ${SOURCES} ${HEADERS})
target_link_libraries(change_led ${WPI_LIB})
target_include_directories(change_led ${CMAKE_CURRENT_LIST_DIR}/include ${WPI_INCLUDE})
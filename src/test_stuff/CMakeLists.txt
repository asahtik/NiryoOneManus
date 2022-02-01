cmake_minimum_required(VERSION 3.0.0)
project(niryo_test_stuff)

set(REPLACEMENTS ${CMAKE_CURRENT_SOURCE_DIR}/src/ros_replacements)

add_executable(test_main ${CMAKE_CURRENT_LIST_DIR}/src/test_main.cpp ${REPLACEMENTS}/ros_replacements/ros_time_repl.hpp)

target_link_libraries(test_main manus_interface)
target_include_directories(test_main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/manus_interface/include ${REPLACEMENTS} manus_interface)

add_dependencies(test_main manus_interface)
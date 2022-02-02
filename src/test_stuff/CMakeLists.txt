cmake_minimum_required(VERSION 3.0.0)
project(niryo_test_stuff)

add_executable(test_main ${CMAKE_CURRENT_LIST_DIR}/src/test_main.cpp)

target_link_libraries(test_main ros_replacements manus_interface)
target_include_directories(test_main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/manus_interface/include ros_replacements manus_interface)

add_dependencies(test_main ros_replacements manus_interface)
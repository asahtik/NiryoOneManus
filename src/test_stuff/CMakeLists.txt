cmake_minimum_required(VERSION 3.0.0)
project(niryo_test_stuff)

set(REPLACEMENTS ${CMAKE_CURRENT_SOURCE_DIR}/src/ros_replacements)
set(INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src/manus_interface)

# add_executable(test_main ${CMAKE_CURRENT_LIST_DIR}/src/test_main.cpp ${REPLACEMENTS}/include/ros_replacements/ros_time_repl.h ${INTERFACE}/include/manus_interface/manus_interface.h)

# target_link_libraries(test_main ros_replacements manus_interface)
# target_include_directories(test_main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/manus_interface/include ros_replacements manus_interface)

# add_dependencies(test_main ros_replacements manus_interface)

add_executable(test_undefined ${CMAKE_CURRENT_LIST_DIR}/src/test_undefined.cpp)
target_link_libraries(test_undefined ros_replacements)
target_include_directories(test_undefined PRIVATE ros_replacements)
add_dependencies(test_undefined ros_replacements)
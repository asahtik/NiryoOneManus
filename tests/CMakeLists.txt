cmake_minimum_required(VERSION 3.0.0)
project(manipulator_test VERSION 0.1.0)

find_package(manus REQUIRED)
find_package(echolib REQUIRED)
FIND_PACKAGE(yaml-cpp REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Werror")

add_executable(test_manipulator test_manipulator.cpp)

target_link_libraries(test_manipulator ${manus_LIBRARIES} ${echolib_LIBRARIES} ${YAML_CPP_LIBRARIES})
target_include_directories(test_manipulator PRIVATE ${manus_INCLUDE_DIRS} ${echolib_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
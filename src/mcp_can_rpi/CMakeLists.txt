################################################################################
# Set variables
################################################################################
set(REPLACEMENTS ${CMAKE_CURRENT_SOURCE_DIR}/src/ros_replacements)

set(SOURCES 
  ${CMAKE_CURRENT_LIST_DIR}/src/mcp_can_rpi.cpp
)
set(HEADERS
  ${CMAKE_CURRENT_LIST_DIR}/include/mcp_can_rpi/mcp_can_dfs_rpi.h
  ${CMAKE_CURRENT_LIST_DIR}/include/mcp_can_rpi/mcp_can_rpi.h
)

###########
## Build ##
###########

add_library(mcp_can_rpi
  ${SOURCES}
  ${HEADERS}
  ${REPLACEMENTS}/ros_replacements/status_output.hpp
)
target_include_directories(mcp_can_rpi PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include ${REPLACEMENTS})

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if (${ARCHITECTURE} MATCHES "arm")
    message(STATUS "wiringPi library is required for mcp_can_rpi (ARM processor)")
    target_link_libraries(mcp_can_rpi
        -lwiringPi
    )
else()
    message(STATUS "wiringPi library not required")
endif()

#############
## Install ##
#############
# install(TARGETS mcp_can_rpi
#   DESTINATION lib
# )
# install(FILES ${HEADERS} DESTINATION lib/include)


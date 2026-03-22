cmake_minimum_required(VERSION 3.10)
project(leap_publisher C)

# Adjust these paths to match your Leap Motion SDK install location
# Default Ultraleap SDK path on Linux: /usr/share/ultraleap or /opt/ultraleap
find_path(LEAPC_INCLUDE_DIR LeapC.h
    PATHS
    /usr/include/ultraleap
    /usr/local/include/ultraleap
    /opt/ultraleap/LeapSDK/include
    /usr/include
    /usr/local/include
)

find_library(LEAPC_LIBRARY
    NAMES LeapC
    PATHS
    /usr/lib
    /usr/local/lib
    /opt/ultraleap/LeapSDK/lib/x64
    /opt/ultraleap/LeapSDK/lib/aarch64
)

if(NOT LEAPC_INCLUDE_DIR OR NOT LEAPC_LIBRARY)
    message(FATAL_ERROR
        "LeapC not found. Install the Ultraleap SDK and set LEAPC_INCLUDE_DIR "
        "and LEAPC_LIBRARY manually if needed.")
endif()

message(STATUS "LeapC include: ${LEAPC_INCLUDE_DIR}")
message(STATUS "LeapC library: ${LEAPC_LIBRARY}")

add_executable(leap_publisher leap_publisher.c)
target_include_directories(leap_publisher PRIVATE ${LEAPC_INCLUDE_DIR})
target_link_libraries(leap_publisher ${LEAPC_LIBRARY} m)

install(TARGETS leap_publisher DESTINATION bin)

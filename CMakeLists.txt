cmake_minimum_required(VERSION 3.18)
project(AirConditionMonitoring)

# BSEC library base path
set(BSEC_INCLUDE_PATH ${CMAKE_SOURCE_DIR}/BSEC2.4.0.0_Generic_Release_23012023/algo/normal_version/inc)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(air_condition_monitoring src/main.cpp)

target_include_directories(air_condition_monitoring PRIVATE ${BSEC_INCLUDE_PATH})

if (WIN32)
    message(STATUS "Building on Windows")
    # Set library folders for Windows
  #  link_directories(${CMAKE_SOURCE_DIR}/BSEC2.4.0.0_Generic_Release_23012023/algo/normal_version/bin/gcc/windows/x64)
elseif (UNIX)
    message(STATUS "Building on Linux/Unix")
    # Set library folders for Linux/Unix
 #   link_directories(${CMAKE_SOURCE_DIR}/BSEC2.4.0.0_Generic_Release_23012023/algo/normal_version/bin/RaspberryPi/PiThree_ArmV8)
else ()
    message(FATAL_ERROR "Unsupported operating system")
endif ()

#target_link_libraries(air_condition_monitoring PRIVATE libalgobsec)

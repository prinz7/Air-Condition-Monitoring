cmake_minimum_required(VERSION 3.18)

project(AirConditionMonitoring)

# BSEC library base path
set(BSEC_INCLUDE_PATH ${CMAKE_SOURCE_DIR}/BSEC2.4.0.0_Generic_Release_23012023/algo/normal_version/inc)
set(BME_INCLUDE_PATH ${CMAKE_SOURCE_DIR}/BSEC2.4.0.0_Generic_Release_23012023/examples/BSEC_Integration_Examples/src/bme68x)
set(BME_CONFIG_INCLUDE_PATH ${CMAKE_SOURCE_DIR}/BSEC2.4.0.0_Generic_Release_23012023/config/bme688/bme688_sel_33v_3s_4d)

if (WIN32)
    message(STATUS "Building on Windows")
    set(BSEC_LIBRARY_PATH ${CMAKE_SOURCE_DIR}/BSEC2.4.0.0_Generic_Release_23012023/algo/normal_version/bin/gcc/windows/x64/libalgobsec.lib)
elseif (UNIX)
    message(STATUS "Building on Linux/Unix")
    set(BSEC_LIBRARY_PATH ${CMAKE_SOURCE_DIR}/BSEC2.4.0.0_Generic_Release_23012023/algo/normal_version/bin/RaspberryPi/PiThree_ArmV8/libalgobsec.a)
else ()
    message(FATAL_ERROR "Unsupported operating system")
endif ()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(bme_library ${BME_INCLUDE_PATH}/bme68x.c ${BME_CONFIG_INCLUDE_PATH}/bsec_selectivity.c)
target_include_directories(bme_library PUBLIC ${BME_INCLUDE_PATH} ${BME_CONFIG_INCLUDE_PATH})

find_library(BCM2835_LIBRARY bcm2835)
if(BCM2835_LIBRARY)
    message(STATUS "Found bcm2835 library: ${BCM2835_LIBRARY}")
else()
    message(FATAL_ERROR "bcm2835 library not found")
endif()

add_executable(air_condition_monitoring src/main.cpp)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(STATUS "Using GCC")

    # Add GCC-specific flags
    target_compile_options(air_condition_monitoring PRIVATE -Wall -Wextra -Wpedantic -O3)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    message(STATUS "Using Visual Studio")

    # Add Visual Studio-specific flags
    target_compile_options(air_condition_monitoring PRIVATE /W4)
else()
    message(FATAL_ERROR "Unsupported compiler")
endif()

target_include_directories(air_condition_monitoring PRIVATE ${BSEC_INCLUDE_PATH} ${BME_INCLUDE_PATH} ${BME_CONFIG_INCLUDE_PATH} include)
target_link_libraries(air_condition_monitoring PRIVATE ${BSEC_LIBRARY_PATH} bme_library ${BCM2835_LIBRARY})

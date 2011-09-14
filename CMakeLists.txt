#=============================================================================#
# Author: Sebastian Rohde                                                      #
# Date:   30.08.2011                                                          #
#=============================================================================#

set(CMAKE_MODULE_PATH    ${CMAKE_SOURCE_DIR}/cmake/modules)  # CMake module search path
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Arduino.cmake) # Arduino Toolchain

set (CMAKE_CXX_SOURCE_FILE_EXTENSIONS pde)

cmake_minimum_required(VERSION 2.8)
#====================================================================#
#  Setup Project                                                     #
#====================================================================#
project(ArdupilotOne C CXX)

find_package(Arduino 22 REQUIRED)

add_subdirectory(libraries)
add_subdirectory(ardupilotone)
#add_subdirectory(testtool)

PRINT_BOARD_SETTINGS(mega2560)



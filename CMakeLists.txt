# Author: Sebastian Rohde, James Goppert                                           

# cmake setup
cmake_minimum_required(VERSION 2.8)
set(CMAKE_MODULE_PATH    ${CMAKE_SOURCE_DIR}/cmake/modules ${CMAKE_SOURCE_DIR}/cmake/arkcmake)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Arduino.cmake)
set (CMAKE_CXX_SOURCE_FILE_EXTENSIONS pde)

# project
project(ArdupilotOne C CXX)

# set these for release
set(APPLICATION_VERSION_MAJOR "0")
set(APPLICATION_VERSION_MINOR "1")
set(CMAKE_CXXFLAGS "-Wformat -Wall -Wshadow -Wpointer-arith -Wcast-align -Wwrite-strings -Wformat=2 -Wno-reorder --mcall-prologues -ffunction-sections -fdata-sections -fno-exceptions")
set(APPLICATION_VERSION_PATCH "0")

# cmake includes
include(CPack)

# cmake settigns
set(APPLICATION_NAME ${PROJECT_NAME})
set(APPLICATION_VERSION "${APPLICATION_VERSION_MAJOR}.${APPLICATION_VERSION_MINOR}.${APPLICATION_VERSION_PATCH}")

# macros
include(MacroEnsureOutOfSourceBuild)

# disallow in-source build
include(MacroEnsureOutOfSourceBuild)
macro_ensure_out_of_source_build("${PROJECT_NAME} requires an out of source build. 
Please create a separate build directory and run 'cmake /path/to/${PROJECT_NAME} [options]' there.")

# cpack settings
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "universal autopilot system for ArduPilotMega")
set(CPACK_PACKAGE_VENDOR "arkTools")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "james.goppert@gmail.com")
set(CPACK_PACKAGE_CONTACT "james.goppert@gmail.com")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_SOURCE_DIR}/README.txt")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/COPYING.txt")
set(CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README.txt")
set(CPACK_PACKAGE_VERSION_MAJOR "${APPLICATION_VERSION_MAJOR}") 
set(CPACK_PACKAGE_VERSION_MINOR "${APPLICATION_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${APPLICATION_VERSION_PATCH}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "CMake ${CMake_VERSION_MAJOR}.${CMake_VERSION_MINOR}")
set(CPACK_SET_DESTDIR TRUE)
set(CPACK_SOURCE_IGNORE_FILES ${CPACK_SOURCE_IGNORE_FILES}
	/.git/;/build/;~$;.*\\\\.bin$;.*\\\\.swp$)
set(CPACK_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# dependencies
find_package(Arduino 22 REQUIRED)

# subdirectories
add_subdirectory(libraries)
add_subdirectory(ardupilotone)
#add_subdirectory(testtool)

option(BOARD "The avr board you are using (mega/mega2560)." mega2560)

PRINT_BOARD_SETTINGS("${BOARD}")

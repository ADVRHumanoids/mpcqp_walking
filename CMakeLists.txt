# 
#  Copyright (C) 2017 cogimon
#  Author: Enrico Mingo Hoffman
#  email: enrico.mingo@iit.it
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>
#

cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)
include(ExternalProject)
PROJECT(mpcqp_walking)

FIND_PACKAGE(YCM REQUIRED)
FIND_PACKAGE(Boost REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wmaybe-uninitialized -Wuninitialized")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

INCLUDE_DIRECTORIES(include ${EIGEN3_INCLUDE_DIR})

set(mpcqp_walking_sources src/linear_inverted_pendulum_model.cpp
                          src/state_machine.cpp
                          src/abstract_variable.cpp
                          src/tools.cpp
                          src/model_preview_control.cpp)

ADD_LIBRARY(mpcqp_walking SHARED ${mpcqp_walking_sources})

TARGET_LINK_LIBRARIES(mpcqp_walking ${Boost_LIBRARIES})

########################################################################
# use YCM to export mpcqp_walking so that it can be found using find_package #
########################################################################

set(VARS_PREFIX "mpcqp_walking")

set(mpcqp_walking_MAJOR_VERSION 0)
set(mpcqp_walking_MINOR_VERSION 0)
set(mpcqp_walking_PATCH_VERSION 1)
set(mpcqp_walking_VERSION ${mpcqp_walking_MAJOR_VERSION}.${mpcqp_walking_MINOR_VERSION}.${mpcqp_walking_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(mpcqp_walking PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                            "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(mpcqp_walking PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                SOVERSION ${${VARS_PREFIX}_VERSION})

target_link_libraries(mpcqp_walking LINK_PUBLIC )

install(DIRECTORY include/ 
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS mpcqp_walking  
        EXPORT mpcqp_walking
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)
        
#enabling it will add all idynutils dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS mpcqp_walking)


include(InstallBasicPackageFiles)
install_basic_package_files(mpcqp_walking   VARS_PREFIX ${VARS_PREFIX}
                                        VERSION ${${VARS_PREFIX}_VERSION}
                                        COMPATIBILITY SameMajorVersion
                                        TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                        NO_CHECK_REQUIRED_COMPONENTS_MACRO)

include(AddUninstallTarget)

## TESTS
enable_testing()
add_subdirectory(tests)

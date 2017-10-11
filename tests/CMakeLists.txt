cmake_minimum_required(VERSION 2.8.11)
include(ExternalProject)

set(PROJECTNAME tests)
project(${PROJECTNAME})

find_package(GTest)
if (NOT GTEST_FOUND)
    set(gTestSource "/usr/src/gtest")
    ExternalProject_Add(GTest-ext SOURCE_DIR ${gTestSource}
                                  PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                                  INSTALL_COMMAND "")
    set(GTEST_LIB_DIRS "${CMAKE_CURRENT_BINARY_DIR}/external/src/GTest-ext-build/")
    set(GTEST_INCLUDE_DIRS ${gTestSource})
    set(GTEST_BOTH_LIBRARIES gtest gtest_main)
endif()

include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR} )
enable_testing()


add_custom_command( TARGET mpcqp_walking POST_BUILD
           COMMAND ${CMAKE_CTEST_COMMAND}
           MAIN_DEPENDENCY mpcqp_walking
           DEPENDS
           testMPC)


SET(TestLibs mpcqp_walking ${GTEST_BOTH_LIBRARIES})

ADD_EXECUTABLE(testMPC     mpc_test.cpp)
TARGET_LINK_LIBRARIES(testMPC ${TestLibs} pthread)
add_dependencies(testMPC GTest-ext mpcqp_walking)

add_test(NAME mpc_test COMMAND testMPC)

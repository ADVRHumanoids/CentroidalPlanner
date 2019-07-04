cmake_minimum_required(VERSION 2.8.11)
include(ExternalProject)

project(centroida_planner_tests C CXX)


# compite GTest
find_package(GTest)
find_package(Threads)
if(NOT GTEST_FOUND)
    set(gTestSource "/usr/src/gtest")
    ExternalProject_Add(GTest-ext SOURCE_DIR ${gTestSource}
                                  PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                                  INSTALL_COMMAND "")
    set(GTEST_LIB_DIRS "${CMAKE_CURRENT_BINARY_DIR}/external/src/GTest-ext-build/")
    set(GTEST_INCLUDE_DIRS ${gTestSource})
    set(GTEST_BOTH_LIBRARIES gtest gtest_main)
endif()

add_definitions(-DCPL_TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/configs/")

set(TestLibs Cpl ${GTEST_BOTH_LIBRARIES})
include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

add_executable(TestBasic TestBasic.cpp)
target_link_libraries(TestBasic ${TestLibs})
add_dependencies(TestBasic GTest-ext Cpl)
add_test(NAME TestBasic COMMAND TestBasic)


project(centroidal_planner)
cmake_minimum_required(VERSION 3.0.0)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroInstallLib)


find_package(XBotInterface REQUIRED)
find_package(XCM QUIET)
find_package(ifopt REQUIRED)
find_package(OpenSoT REQUIRED)
find_package(ReflexxesTypeII REQUIRED)
find_package(cartesian_interface REQUIRED)
find_package(alglib QUIET)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        std_msgs
                                        geometry_msgs
                                        sensor_msgs
                                        actionlib
                                        robot_state_publisher
                                        message_generation
                                        interactive_markers
                                        tf tf_conversions)

include_directories(include
                    ${OpenSoT_INCLUDE_DIRS}
                    ${XBotInterface_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${ReflexxesTypeII_INCLUDE_DIRS}
                    ${alglib_INCLUDE_DIRS}
                    ${XCM_INCLUDE_DIRS}
                    )

# C++ 11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


catkin_package(
   INCLUDE_DIRS include 
)


SET(IFOPT_SOURCES src/Constraints/CentroidalStatics.cpp
                  src/Constraints/FrictionCone.cpp
                  src/Constraints/EnvironmentConstraint.cpp
                  src/MinimizeCentroidalVariables.cpp
                  )
                  
SET(ENVIRONMENT_SOURCES src/Superquadric.cpp
                  )                  
                  
ADD_LIBRARY(Centroidal SHARED
                       ${IFOPT_SOURCES}
                       ${ENVIRONMENT_SOURCES})
                    
TARGET_LINK_LIBRARIES(Centroidal PUBLIC
                                 ${OpenSoT_LIBRARIES}
                                 ${XBotInterface_LIBRARIES}
                                 ${catkin_LIBRARIES}
                                 ${ReflexxesTypeII_LIBRARIES}
                                 ${cartesian_interface_LIBRARIES}  
                                 ifopt::ifopt_ipopt)


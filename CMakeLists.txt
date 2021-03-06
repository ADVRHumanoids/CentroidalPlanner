project(centroidal_planner)
cmake_minimum_required(VERSION 3.0.0)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroInstallLib)


find_package(XCM QUIET)
find_package(ifopt REQUIRED)
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
                    ${EIGEN3_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
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

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
   INCLUDE_DIRS include 
   LIBRARIES Cpl

)

SET(IFOPT_SOURCES src/Constraints/CentroidalStatics.cpp
                  src/Constraints/FrictionCone.cpp
                  src/Constraints/EnvironmentConstraint.cpp
                  src/Constraints/EnvironmentNormal.cpp
                  src/MinimizeCentroidalVariables.cpp
                  src/Variable3D.cpp
                  src/CplProblem.cpp
                  )
                  
SET(ENVIRONMENT_SOURCES src/Superquadric.cpp
                        src/Ground.cpp
                        )   
                  
SET(CPL_SOURCES src/CentroidalPlanner.cpp
                src/CoMPlanner.cpp
                ) 
                  
ADD_LIBRARY(Cpl SHARED
                       ${IFOPT_SOURCES}
                       ${ENVIRONMENT_SOURCES}
                       ${CPL_SOURCES}
                       )

add_dependencies(Cpl ${PROJECT_NAME}_generate_messages)
                    
TARGET_LINK_LIBRARIES(Cpl PUBLIC
                                 ${catkin_LIBRARIES}
                                 ifopt::ifopt_ipopt)



INSTALL(TARGETS Cpl
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h")
              

option(BUILD_TESTS FALSE)
if(${BUILD_TESTS})
    add_subdirectory(tests/)
endif()

add_subdirectory(bindings/python/)


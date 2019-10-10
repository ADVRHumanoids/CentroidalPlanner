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

include_directories(include/
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

add_service_files(
  FILES
  SetLiftedContacts.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
   INCLUDE_DIRS include 
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
                src/Utils.cpp
                ) 

SET(FORZA_GIUSTA src/forza_giusta/ForzaGiustaFeedForward.cpp
                )

ADD_LIBRARY(Cpl SHARED
            ${IFOPT_SOURCES}
            ${ENVIRONMENT_SOURCES}
            ${CPL_SOURCES}
            )
                    
TARGET_LINK_LIBRARIES(Cpl PUBLIC
                      ${OpenSoT_LIBRARIES}
                      ${XBotInterface_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${ReflexxesTypeII_LIBRARIES}
                      ${cartesian_interface_LIBRARIES}
                      ifopt::ifopt_ipopt
                      )

add_executable(forza_giusta_node src/forza_giusta/ForzaGiustaFeedForward.cpp)

target_link_libraries(forza_giusta_node ${OpenSoT_LIBRARIES}
                                        ${XBotInterface_LIBRARIES}
                                        ${catkin_LIBRARIES}
                                        ${ReflexxesTypeII_LIBRARIES}
                                        ${cartesian_interface_LIBRARIES})

install(TARGETS forza_giusta_node forza_giusta_node
                                  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
                                  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
                                  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
                                  )


INSTALL(TARGETS Cpl 
        DESTINATION lib)
           
## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
                                                      
## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY cartesio_stack/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cartesio_stack
  FILES_MATCHING PATTERN "*.yaml"
)        

install(DIRECTORY include/forza_giusta/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

option(BUILD_TESTS FALSE)
if(${BUILD_TESTS})
    add_subdirectory(tests/)
endif()



add_subdirectory(bindings/python/)

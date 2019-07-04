cmake_minimum_required(VERSION 3.5)

set(PYBIND11_PYTHON_VERSION 2.7 CACHE STRING "")
find_package(pybind11 QUIET)

if(${pybind11_FOUND})

    message(STATUS "Compiling python bindings")
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    pybind11_add_module(pycpl pyCpl.cpp)
    target_link_libraries(pycpl PUBLIC Cpl)

    install(TARGETS pycpl
            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
            
    file(WRITE ${CMAKE_BINARY_DIR}/__init__.py "")
    install(FILES ${CMAKE_BINARY_DIR}/__init__.py DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
            
endif()
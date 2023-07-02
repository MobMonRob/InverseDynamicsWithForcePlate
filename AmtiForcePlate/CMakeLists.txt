cmake_minimum_required(VERSION 3.5.1)

project(AmtiForcePlate
    VERSION 1.0 LANGUAGES CXX
)

#useful for debugging
#SET(CMAKE_VERBOSE_MAKEFILE ON)

set (CMAKE_CXX_STANDARD_REQUIRED 20)

SET (VICON_ROOT "${CMAKE_SOURCE_DIR}/ViconLib1.12")

foreach(src ${SOURCES})
    list(APPEND SOURCES_tmp "${src}.cpp")
endforeach()
set(SOURCES ${SOURCES_tmp})

file(GLOB_RECURSE VICON_SRC FOLLOW_SYMLINKS ${VICON_ROOT} "${VICON_ROOT}/*.cpp")

file(GLOB_RECURSE VICON_H FOLLOW_SYMLINKS ${VICON_ROOT} "${VICON_ROOT}/*.h")

add_executable(forcePlateGetData.out Main.cpp DataStreamClientFacade.cpp ForcePlateDataAcquisition.cpp ForcePlateData.cpp) 

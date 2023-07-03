cmake_minimum_required(VERSION 3.5.1)

project(AmtiForcePlate
    VERSION 1.0 LANGUAGES CXX
)

# Use C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use the VERBOSE option for debugging
# set(CMAKE_VERBOSE_MAKEFILE ON)

# Set the ViconLib directory
set(VICON_ROOT "${CMAKE_SOURCE_DIR}/ViconLib1.12")

# Add all .cpp files in the current directory to the sources list
file(GLOB SOURCES "*.cpp")

# Add the executable and link the sources
add_executable(forcePlateGetData.out ${SOURCES})

# Specify the include directories
target_include_directories(forcePlateGetData.out PRIVATE ${VICON_ROOT})

# Specify the library directory
target_link_directories(forcePlateGetData.out PRIVATE ${VICON_ROOT})

# Use a wildcard to include all required libraries
file(GLOB LIBRARIES "${VICON_ROOT}/*.so.1.75.0" "${VICON_ROOT}/*.so")

# Link the libraries
target_link_libraries(forcePlateGetData.out ${LIBRARIES})

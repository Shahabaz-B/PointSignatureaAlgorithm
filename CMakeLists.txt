# min. version of cmake required for this project
cmake_minimum_required( VERSION 3.10)

# name of the project
project(pointSignatureAlgortihm)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# collect all the .cxx needed for executable 
filE(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
filE(GLOB EXAMPLESRC ${CMAKE_CURRENT_SOURCE_DIR}/example/*.cpp)

# Where to find .h/.hpp files
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include" )

# Executable target name and srouce
add_executable (${PROJECT_NAME} ${SOURCES} ${EXAMPLESRC})

# Link dependencies
target_link_libraries (${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/flann/build/lib/libflann.so)

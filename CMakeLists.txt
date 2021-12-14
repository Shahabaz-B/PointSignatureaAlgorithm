# min. version of cmake required for this project
cmake_minimum_required( VERSION 3.10)

# name of the project
project(pointSignatureAlgortihm)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# collect all the .cxx needed for executable 
filE(GLOB SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/libs/pointCloudFilters/src/*" )

filE(GLOB EXAMPLESRC ${CMAKE_CURRENT_SOURCE_DIR}/example/*.cpp)

# Executable target name and srouce
add_executable (${PROJECT_NAME} ${SOURCES} ${EXAMPLESRC})

# Link dependencies
# target_link_libraries (${PROJECT_NAME} )
target_include_directories(${PROJECT_NAME} PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}/libs/pointCloudFilters/include
${CMAKE_CURRENT_SOURCE_DIR}/include
)

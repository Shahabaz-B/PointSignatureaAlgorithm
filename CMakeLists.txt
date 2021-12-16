# min. version of cmake required for this project
cmake_minimum_required( VERSION 3.10)

# name of the project
project(pointSignatureAlgortihm)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build/lib)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# collect all the .cxx needed for executable 
filE(GLOB LIBSOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" )

add_subdirectory(thirdparty/pointCloudFilters)
add_subdirectory(examples)

add_library( ${PROJECT_NAME}_shared SHARED ${LIBSOURCES} )
target_link_libraries( ${PROJECT_NAME}_shared PUBLIC
pointCloudFilters_shared)

target_include_directories( ${PROJECT_NAME}_shared PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pointCloudFilters/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

add_library(${PROJECT_NAME}_static STATIC ${LIBSOURCES})
target_link_libraries( ${PROJECT_NAME}_static PUBLIC
pointCloudFilters_static)

target_include_directories( ${PROJECT_NAME}_static PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pointCloudFilters/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
)
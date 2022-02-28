project(pointSignature)

file(GLOB SOURCES
"src/*.cpp" "${CMAKE_SOURCE_DIR}/src/*.cpp"
)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(
	${PROJECT_NAME}
	${SOURCES}
)
# Link dependencies
target_link_libraries (${PROJECT_NAME}
PUBLIC
	pointCloudFilters_shared
)

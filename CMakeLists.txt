cmake_minimum_required(VERSION 3.16)

project(pointcloud)

find_package(PCL 1.12 REQUIRED)
find_package(Threads REQUIRED)

fins_add_node(${PROJECT_NAME} nodes.cpp)

target_compile_definitions(${PROJECT_NAME} PRIVATE BOOST_ALLOW_DEPRECATED_HEADERS)

target_include_directories(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE
	${PCL_LIBRARIES}
	Threads::Threads
)

add_definitions(${PCL_DEFINITIONS})
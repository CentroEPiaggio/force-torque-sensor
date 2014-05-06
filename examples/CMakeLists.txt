cmake_minimum_required(VERSION 2.8)

add_executable(simple_reader 
	simple_reader.cpp
)

add_dependencies(simple_reader 
	FTSensor
)

target_link_libraries (simple_reader 
	FTSensor
)
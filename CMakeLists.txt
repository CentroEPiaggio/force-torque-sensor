cmake_minimum_required(VERSION 2.8.3)
project(force_torque_sensor)

if (CMAKE_VERSION VERSION_LESS "3.1")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(LibXml2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  message_generation 
  std_msgs
  geometry_msgs 
)

catkin_package()


include_directories(include 
  ${LIBXML2_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(FTSensor
  src/FTSensor.cpp
  )

add_executable(ft_sensor_hw 
  src/ft_sensor_hw.cpp
)

add_dependencies(ft_sensor_hw 
  FTSensor
)

target_link_libraries(ft_sensor_hw
  FTSensor
  ${LIBXML2_LIBRARIES}
  ${catkin_LIBRARIES}
)
cmake_minimum_required(VERSION 2.8.3)
project(force_torque_sensor)

find_package(LibXml2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  message_generation 
  std_msgs
  geometry_msgs 
  controller_interface
  controller_manager
  hardware_interface
  force_torque_sensor_controller
  
)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS hardware_interface controller_interface)


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
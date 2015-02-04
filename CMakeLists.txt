cmake_minimum_required(VERSION 2.8.3)
project(force_torque_sensor)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  message_generation 
  std_msgs 
  geometry_msgs 
  tf
)

find_package(LibXml2 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ftsensor
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include 
  ${LIBXML2_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(FTSensor
  src/FTSensor.cpp
  )

add_executable(ftsensor_CLIENT 
  src/FT_sensor_node.cpp
)

add_dependencies(ftsensor_CLIENT 
  FTSensor
)

target_link_libraries(ftsensor_CLIENT
  FTSensor
  ${LIBXML2_LIBRARIES}
  ${catkin_LIBRARIES}
)
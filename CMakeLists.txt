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
  filters
  gazebo_ros
  #gazebo_plugins
)

catkin_package(
INCLUDE_DIRS include
  LIBRARIES force_torque_sensor_sim_ros_control
  CATKIN_DEPENDS roscpp std_msgs hardware_interface controller_manager 
  DEPENDS system_lib)


include_directories(include 
  include
  ${LIBXML2_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)


add_library(FTSensor
  src/FTSensor.cpp
)

add_library(force_torque_sensor_sim_ros_control
  src/force_torque_sensor_sim_ros_control.cpp
)

add_library(force_torque_sensor_filter
  src/trans_function_filter.cpp
)
add_library(imu_sensor_filter
  src/imu_trans_function_filter.cpp
)


add_executable(ft_sensor_hw 
  src/ft_sensor_hw.cpp
)

add_executable(force_torque_sensor_sim_ros_control_node src/force_torque_sensor_sim_ros_control.cpp)
add_executable(force_torque_sensor_filter_node src/trans_function_filter.cpp)
add_executable(imu_sensor_filter_node src/imu_trans_function_filter.cpp)

add_dependencies(ft_sensor_hw 
  FTSensor
)

add_dependencies(force_torque_sensor_sim_ros_control_node 
  force_torque_sensor_sim_ros_control
)

add_dependencies(force_torque_sensor_filter_node 
  force_torque_sensor_filter
)

add_dependencies(imu_sensor_filter_node 
  imu_sensor_filter
)

target_link_libraries(ft_sensor_hw
  FTSensor
  ${LIBXML2_LIBRARIES}
  ${catkin_LIBRARIES}
)



target_link_libraries(force_torque_sensor_sim_ros_control_node
  ${catkin_LIBRARIES}
)
target_link_libraries(force_torque_sensor_filter_node
  ${catkin_LIBRARIES}
)
target_link_libraries(imu_sensor_filter_node
  ${catkin_LIBRARIES}
)


add_dependencies(force_torque_sensor_sim_ros_control_node ${catkin_EXPORTED_TARGETS})
add_dependencies(force_torque_sensor_filter_node  ${catkin_EXPORTED_TARGETS})
add_dependencies(imu_sensor_filter_node  ${catkin_EXPORTED_TARGETS})



add_library(gazebo_ros_imu_sensor src/gazebo_ros_imu_sensor.cpp)
target_link_libraries(gazebo_ros_imu_sensor ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
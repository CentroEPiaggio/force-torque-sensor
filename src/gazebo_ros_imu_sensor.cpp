/* Copyright [2015] [Alessandro Settimi]
 * 
 * email: ale.settimi@gmail.com
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/

#include <IMUSensor/gazebo_ros_imu_sensor.h>
#include <iostream>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/physics/World.hh>

GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboRosImuSensor)

gazebo::GazeboRosImuSensor::GazeboRosImuSensor(): SensorPlugin()
{
  accelerometer_data = math::Vector3(0, 0, 0);
  gyroscope_data = math::Vector3(0, 0, 0);
  orientation = math::Quaternion(1,0,0,0);
  seed=0;
}

void gazebo::GazeboRosImuSensor::Load(gazebo::sensors::SensorPtr sensor_, sdf::ElementPtr sdf_)
{
  sdf=sdf_;
  sensor=dynamic_cast<gazebo::sensors::ImuSensor*>(sensor_.get());
  sensor->SetActive(true);

  if(!LoadParameters())
  {
    ROS_FATAL("Error Loading Parameters!");
    return;
  }

  if (!ros::isInitialized()) //check if ros is initialized properly
  {
    ROS_FATAL("ROS has not been initialized!");
    return;
  }

  node = new ros::NodeHandle(this->robot_namespace);

  imu_data_publisher = node->advertise<sensor_msgs::Imu>(topic_name,1);

  connection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosImuSensor::UpdateChild, this, _1));

  last_time = sensor->GetLastUpdateTime();
}

void gazebo::GazeboRosImuSensor::UpdateChild(const gazebo::common::UpdateInfo &/*_info*/)
{
  common::Time current_time = sensor->GetLastUpdateTime();

  if(update_rate>0 && (current_time-last_time).Double() < 1.0/update_rate) //update rate check
    return;

  if(imu_data_publisher.getNumSubscribers() > 0)
  {
    orientation = offset.rot*sensor->GetOrientation(); //applying offsets to the orientation measurement
    accelerometer_data = sensor->GetLinearAcceleration();
    gyroscope_data = sensor->GetAngularVelocity();
    imu_msg.header.stamp.sec = current_time.sec;
    imu_msg.header.stamp.nsec = current_time.nsec;

    imu_msg.header.frame_id=body_name;
    //Guassian noise is applied to all measurements
    imu_msg.orientation.x = orientation.x + GuassianKernel(0,gaussian_noise);
    imu_msg.orientation.y = orientation.y + GuassianKernel(0,gaussian_noise);
    imu_msg.orientation.z = orientation.z + GuassianKernel(0,gaussian_noise);
    imu_msg.orientation.w = orientation.w + GuassianKernel(0,gaussian_noise);

    imu_msg.linear_acceleration.x = accelerometer_data.x + GuassianKernel(0,gaussian_noise);
    imu_msg.linear_acceleration.y = accelerometer_data.y + GuassianKernel(0,gaussian_noise);
    imu_msg.linear_acceleration.z = accelerometer_data.z + GuassianKernel(0,gaussian_noise);

    imu_msg.angular_velocity.x = gyroscope_data.x + GuassianKernel(0,gaussian_noise);
    imu_msg.angular_velocity.y = gyroscope_data.y + GuassianKernel(0,gaussian_noise);
    imu_msg.angular_velocity.z = gyroscope_data.z + GuassianKernel(0,gaussian_noise);

    //covariance is related to the Gaussian noise
    double gn2 = gaussian_noise*gaussian_noise;
    imu_msg.orientation_covariance[0] = gn2;
    imu_msg.orientation_covariance[4] = gn2;
    imu_msg.orientation_covariance[8] = gn2;
    imu_msg.angular_velocity_covariance[0] = gn2;
    imu_msg.angular_velocity_covariance[4] = gn2;
    imu_msg.angular_velocity_covariance[8] = gn2;
    imu_msg.linear_acceleration_covariance[0] = gn2;
    imu_msg.linear_acceleration_covariance[4] = gn2;
    imu_msg.linear_acceleration_covariance[8] = gn2;

    //publishing data
    imu_data_publisher.publish(imu_msg);

    ros::spinOnce();
  }

  last_time = current_time;
}

double gazebo::GazeboRosImuSensor::GuassianKernel(double mu, double sigma)
{
  // generation of two normalized uniform random variables
  double U1 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);
  double U2 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);

  // using Box-Muller transform to obtain a varaible with a standard normal distribution
  double Z0 = sqrt(-2.0 * ::log(U1)) * cos(2.0*M_PI * U2);

  // scaling
  Z0 = sigma * Z0 + mu;
  return Z0;
}

bool gazebo::GazeboRosImuSensor::LoadParameters()
{
  //loading parameters from the sdf file

  //NAMESPACE
  if (sdf->HasElement("robotNamespace"))
  {
    robot_namespace =  "/" + sdf->Get<std::string>("robotNamespace") +"/";
    ROS_INFO_STREAM("<robotNamespace> set to: "<<robot_namespace);
  }
  else
  {
    std::string scoped_name = sensor->GetParentName(); //HACK to find the model name
    std::size_t it = scoped_name.find("::");

    robot_namespace = "/" +scoped_name.substr(0,it)+"/";
    ROS_WARN_STREAM("missing <robotNamespace>, set to default: " << robot_namespace);
  }

  //TOPIC
  if (sdf->HasElement("topicName"))
  {
    topic_name =  robot_namespace + sdf->Get<std::string>("topicName");
    ROS_INFO_STREAM("<topicName> set to: "<<topic_name);
  }
  else
  {
    topic_name = robot_namespace + "/imu_data";
    ROS_WARN_STREAM("missing <topicName>, set to /namespace/default: " << topic_name);
  }

  //BODY, UNUSED
  if (sdf->HasElement("bodyName"))
  {
    body_name =  sdf->Get<std::string>("bodyName");
    ROS_INFO_STREAM("<bodyName> set to: "<<body_name);
  }
  else
  {
    ROS_FATAL("missing <bodyName>, cannot proceed");
    return false;
  }

  //UPDATE RATE
  if (sdf->HasElement("updateRateHZ"))
  {
    update_rate =  sdf->Get<double>("updateRateHZ");
    ROS_INFO_STREAM("<updateRateHZ> set to: " << update_rate);
  }
  else
  {
    update_rate = 1.0;
    ROS_WARN_STREAM("missing <updateRateHZ>, set to default: " << update_rate);
  }

  //NOISE
  if (sdf->HasElement("gaussianNoise"))
  {
    gaussian_noise =  sdf->Get<double>("gaussianNoise");
    ROS_INFO_STREAM("<gaussianNoise> set to: " << gaussian_noise);
  }
  else
  {
    gaussian_noise = 0.0;
    ROS_WARN_STREAM("missing <gaussianNoise>, set to default: " << gaussian_noise);
  }

  //POSITION OFFSET, UNUSED
  if (sdf->HasElement("xyzOffset"))
  {
    offset.pos =  sdf->Get<math::Vector3>("xyzOffset");
    ROS_INFO_STREAM("<xyzOffset> set to: " << offset.pos[0] << ' ' << offset.pos[1] << ' ' << offset.pos[2]);
  }
  else
  {
    offset.pos = math::Vector3(0, 0, 0);
    ROS_WARN_STREAM("missing <xyzOffset>, set to default: " << offset.pos[0] << ' ' << offset.pos[1] << ' ' << offset.pos[2]);
  }

  //ORIENTATION OFFSET
  if (sdf->HasElement("rpyOffset"))
  {
    offset.rot =  sdf->Get<math::Vector3>("rpyOffset");
    ROS_INFO_STREAM("<rpyOffset> set to: " << offset.rot.GetRoll() << ' ' << offset.rot.GetPitch() << ' ' << offset.rot.GetYaw());
  }
  else
  {
    offset.rot = math::Vector3(0, 0, 0);
    ROS_WARN_STREAM("missing <rpyOffset>, set to default: " << offset.rot.GetRoll() << ' ' << offset.rot.GetPitch() << ' ' << offset.rot.GetYaw());
  }

  return true;
}

gazebo::GazeboRosImuSensor::~GazeboRosImuSensor()
{
  if (connection.get())
  {
    gazebo::event::Events::DisconnectWorldUpdateBegin(connection);
    connection = gazebo::event::ConnectionPtr();
  }

  node->shutdown();
}
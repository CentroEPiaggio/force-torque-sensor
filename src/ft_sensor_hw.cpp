#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Empty.h>

#include <string>

// FTSensor class definition
#include "FTSensor/FTSensor.h"

namespace ftsensor {

class FTSensorHW
{
  private:
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    //! The sensor 
    FTSensor* ftsensor_;
    std::string ip_;
    std::string name_;
    std::string type_;
    
    //! Publisher for sensor readings
    ros::Publisher pub_sensor_readings_;

    //! Service for setting the bias
    ros::ServiceServer srv_set_bias_;
   
  public:
    //------------------ Callbacks -------------------
    // Callback for setting bias
    bool setBiasCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

    // Publish the measurements
    void publishMeasurements();

    //! Subscribes to and advertises topics
    FTSensorHW(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {

      priv_nh_.param<std::string>("name", name_, "my_sensor");
      priv_nh_.param<std::string>("type", type_, "nano17");
      priv_nh_.param<std::string>("ip", ip_, "192.168.0.100");

      char* ip = new char[ip_.size() + 1];
      std::copy(ip_.begin(), ip_.end(), ip);
      ip[ip_.size()] = '\0'; // don't forget the terminating 0

      ROS_INFO("FT Sensor config:");
      ROS_INFO_STREAM("ip: " << ip);
      ROS_INFO_STREAM("name: " << name_);
      ROS_INFO_STREAM("type: " << type_);

      // Create a new sensor
      ftsensor_ = new FTSensor();
      // Set ip of FT Sensor
      ftsensor_->setIP(ip);
      // don't forget to free the string after finished using it
      delete[] ip;

      // Init FT Sensor
      ftsensor_->init();
      // Set bias
      ftsensor_->setBias();

      // Advertise topic where readings are published
      pub_sensor_readings_ = nh_.advertise<geometry_msgs::WrenchStamped>(nh_.resolveName("sensor_measurements"), 10);
      
      // Advertise service for setting the bias
      srv_set_bias_ = nh_.advertiseService(nh_.resolveName("tare"), &FTSensorHW::setBiasCallback, this);
    }

    //! Empty stub
    ~FTSensorHW() {}

};

// ToDo: setBias and Tare are different things
// setBias(A B C D E F) to set an external bias manually
// Tare() it set the bias such that all sensor_measurements are zero
bool FTSensorHW::setBiasCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  ftsensor_->setBias();

  return true;
}

void FTSensorHW::publishMeasurements()
{
  geometry_msgs::WrenchStamped ftreadings;
  float measurements[6];
  ftsensor_->getMeasurements(measurements);
  
  ftreadings.wrench.force.x = measurements[0];
  ftreadings.wrench.force.y = measurements[1];
  ftreadings.wrench.force.z = measurements[2];
  ftreadings.wrench.torque.x = measurements[3];
  ftreadings.wrench.torque.y = measurements[4];
  ftreadings.wrench.torque.z = measurements[5];

  ftreadings.header.stamp = ros::Time::now();
  ftreadings.header.frame_id = name_ + "_" + type_ + "_" + "measure";

  pub_sensor_readings_.publish(ftreadings);
}

} // namespace ftsensor

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "ft_sensor_hw");
  ros::NodeHandle nh;
  ros::Rate loop(100);
  ftsensor::FTSensorHW node(nh);

  while(ros::ok())
  {
    node.publishMeasurements();
    ros::spinOnce();  

    loop.sleep();
  }
  return 0;
}
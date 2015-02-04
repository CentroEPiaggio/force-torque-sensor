#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <string>

// FTSensor class definition
#include "FTSensor/FTSensor.h"

namespace ftsensor {

class FTSensorPublisher
{
  private:
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    //! The sensor 
    FTSensor* ftsensor_;
    std::string ip_;
    std::string frame_ft_;
    
    //! Publisher for sensor readings
    ros::Publisher pub_sensor_readings_;

    //! Service for setting the bias
    ros::ServiceServer srv_set_bias_;

    // There always should be a listener and a broadcaster!
    //! A tf transform listener
    //tf::TransformListener tf_listener_;

    //! A tf transform broadcaster
    //tf::TransformBroadcaster tf_broadcaster_;

    //tf::Transform sensor_top_frame_;
    //tf::Transform sensor_bottom_frame_;
    
  public:
    //------------------ Callbacks -------------------
    // Callback for setting bias
    bool setBiasCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

    // Publish the measurements
    void publishMeasurements();

    //! Subscribes to and advertises topics
    FTSensorPublisher(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {

      priv_nh_.param<std::string>("frame_ft", frame_ft_, "/sensor_top_frame");
      priv_nh_.param<std::string>("ip", ip_, "192.168.0.100");

      char* ip = new char[ip_.size() + 1];
      std::copy(ip_.begin(), ip_.end(), ip);
      ip[ip_.size()] = '\0'; // don't forget the terminating 0

      ROS_INFO("ip %s", ip);

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
      pub_sensor_readings_ = nh_.advertise<geometry_msgs::WrenchStamped>(nh_.resolveName("sensor_readings"), 10);
      
      // Advertise service for setting the bias
      srv_set_bias_ = nh_.advertiseService(nh_.resolveName("set_bias"), &FTSensorPublisher::setBiasCallback, this);

      // for now, the transform is constant, it has to be read from a topic comming from the robot for the bottom plate, and from the stewart platform displacement for the top plate
      //sensor_top_frame_.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
      //sensor_top_frame_.setRotation( tf::Quaternion(0.0, 0.0, 0.0, 1.0) );
    }

    //! Empty stub
    ~FTSensorPublisher() {}

};

bool FTSensorPublisher::setBiasCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  ftsensor_->setBias();
}

void FTSensorPublisher::publishMeasurements()
{
  // Recall that this has to be transformed using the stewart platform
  //tf_broadcaster_.sendTransform(tf::StampedTransform(nano_top_frame_, ros::Time::now(), "/world", "/nano_top_frame"));
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
  ftreadings.header.frame_id = frame_ft_;

  pub_sensor_readings_.publish(ftreadings);

  //ROS_INFO("Measured Force: %f %f %f Measured Torque: %f %f %f", measurements[0], measurements[1], measurements[2], measurements[3], measurements[4], measurements[5]);
}

} // namespace ftsensor

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "FT_sensor_node");
  ros::NodeHandle nh;
  ros::Rate loop(100);
  ftsensor::FTSensorPublisher node(nh);

  while(ros::ok())
  {
    node.publishMeasurements();
    ros::spinOnce();  

    loop.sleep();
  }
  return 0;
}
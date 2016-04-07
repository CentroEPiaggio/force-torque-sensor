/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Luca Gemma
 *                      Team Pacman,
 *                      Università di Pisa, Centro Piaggio
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Technische Universität Darmstadt nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/



#include <FTSensor/force_torque_sensor_sim_ros_control.h>

#include <controller_manager/controller_manager.h>
#include <std_msgs/String.h>


namespace force_torque_sensor_sim_ros_control
{
  void ForceTorqueSensorSimRosControl::init(std::string ft_sensor_params_name)
  {
    //ROS_INFO_STREAM("ft_sensor_params_name: "<< ft_sensor_params_name);
    nh_.getParam(ft_sensor_params_name+"/ft_sensor_name", ft_sensor_name);
    nh_.getParam(ft_sensor_params_name+"/ft_sensor_frame_id", ft_sensor_frame_id);
   
    ROS_INFO_STREAM("name: "<< ft_sensor_name);
    ROS_INFO_STREAM("ft_sensor_frame_id: "<< ft_sensor_frame_id);
 


    hardware_interface::ForceTorqueSensorHandle force_torque_handle(ft_sensor_name, ft_sensor_frame_id, force_, torque_);
    
    force_torque_sensor_interface_.registerHandle(force_torque_handle);

    registerInterface(&force_torque_sensor_interface_);
  }

  void ForceTorqueSensorSimRosControl::force_torque_sensor_State(const geometry_msgs::WrenchStamped::ConstPtr &_ws)
  {

    WrenchStamped = *_ws;

    force_[0] = WrenchStamped.wrench.force.x;
    force_[1] = WrenchStamped.wrench.force.y;
    force_[2] = WrenchStamped.wrench.force.z;
    torque_[0] = WrenchStamped.wrench.torque.x;
    torque_[1] = WrenchStamped.wrench.torque.y;
    torque_[2] = WrenchStamped.wrench.torque.z;

   //ROS_INFO("force %f , %f , %f ",(float)force_[0],(float)force_[1],(float)force_[2]);
   //ROS_INFO("torque %f , %f , %f ",(float)torque_[0],(float)torque_[1],(float)torque_[2]);

  }

  ForceTorqueSensorSimRosControl::ForceTorqueSensorSimRosControl(std::string ft_sensor_params_name)
  {
    ros::NodeHandle* rosnode = new ros::NodeHandle();

    rosnode->setCallbackQueue(&subscriber_queue_);

    ros::Time last_ros_time_;
    bool wait = true;
    while (wait)
    {
      last_ros_time_ = ros::Time::now();
      if (last_ros_time_.toSec() > 0)
        wait = false;
    }
    nh_.getParam(ft_sensor_params_name+"/ft_sensor_topic", ft_sensor_topic);
    ROS_INFO_STREAM("ft_sensor_topic: "<< ft_sensor_topic);
      // ros topic subscribtions
    ros::SubscribeOptions Sub_Wrench =
        ros::SubscribeOptions::create<geometry_msgs::WrenchStamped>(
          ft_sensor_topic, 1,boost::bind(&ForceTorqueSensorSimRosControl::force_torque_sensor_State, this, _1),
          ros::VoidPtr(), rosnode->getCallbackQueue());

    Sub_Wrench_ = rosnode->subscribe(Sub_Wrench);
       
    subscriber_spinner_.reset(new ros::AsyncSpinner(1, &subscriber_queue_));
    subscriber_spinner_->start();
   
  }  


  void ForceTorqueSensorSimRosControl::cleanup()
  {
    subscriber_spinner_->stop();
  }

  void ForceTorqueSensorSimRosControl::read(ros::Time time, ros::Duration period)
  {
    //Normally we would read joint angles from hardware here, but we get those via ROS topic as we use Gazebo
  }

}

int main(int argc, char** argv){

  try{
    ROS_INFO("starting");
    ros::init(argc, argv, "force_torque_sensor_sim_ros_control");

    
    
    //sleep(2);

    // Publish sensor user mode

    ros::NodeHandle nh;
    std::string ft_sensor_params_name;

    nh.getParam("ft_sensor_params_name", ft_sensor_params_name);
    ROS_INFO_STREAM("ft_sensor_params_name: "<< ft_sensor_params_name);

    force_torque_sensor_sim_ros_control::ForceTorqueSensorSimRosControl force_torque_sensor_sim_ros_control_interface(ft_sensor_params_name);

    force_torque_sensor_sim_ros_control_interface.init(ft_sensor_params_name);

    ros::NodeHandle controller_nh("force_torque_sensor_controller");

    ros::Publisher pub_user_mode_ = nh.advertise<std_msgs::String>("ft_sensor/control_mode",1,true);

    std_msgs::String msg;
    std::stringstream ss;
    ss << "User";
    msg.data = ss.str();
    pub_user_mode_.publish(msg);

    controller_manager::ControllerManager cm(&force_torque_sensor_sim_ros_control_interface, controller_nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Rate loop_rate(1000);

    ros::Time last_time = ros::Time::now();

    while (ros::ok())
    {


      ros::Time current_time = ros::Time::now();
      ros::Duration elapsed_time = current_time - last_time;
      last_time = current_time;

      force_torque_sensor_sim_ros_control_interface.read(current_time, elapsed_time);

      cm.update(current_time, elapsed_time);
     
      loop_rate.sleep();
    }

    force_torque_sensor_sim_ros_control_interface.cleanup();
  }
  catch(...)
  {
    ROS_ERROR("Unhandled exception!");
    return -1;
  }

  return 0;
}

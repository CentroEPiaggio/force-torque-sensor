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


#ifndef Force_Torque_Sensor_Sim_Ros_Control_hpp___
#define Force_Torque_Sensor_Sim_Ros_Control_hpp___

#include <ros/ros.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/robot_hw.h>

#include <std_msgs/Float64.h>

#include <ros/callback_queue.h>

#include <geometry_msgs/WrenchStamped.h> 



namespace force_torque_sensor_sim_ros_control
{

class Force_Torque_Sensor_Sim_Ros_Control : public hardware_interface::RobotHW
{
public:
    Force_Torque_Sensor_Sim_Ros_Control();

    void cleanup();

    void read(ros::Time time, ros::Duration period);

private:

    void force_torque_sensor_State(const geometry_msgs::WrenchStamped::ConstPtr &_ws);

    ros::NodeHandle nh_;

    boost::shared_ptr<ros::AsyncSpinner> subscriber_spinner_;
    ros::CallbackQueue subscriber_queue_;
    ros::Subscriber Sub_Wrench_;

    hardware_interface::ForceTorqueSensorInterface force_torque_sensor_interface_;

    geometry_msgs::WrenchStamped WrenchStamped;

    double force_[3];
    double torque_[3];


};

}

#endif

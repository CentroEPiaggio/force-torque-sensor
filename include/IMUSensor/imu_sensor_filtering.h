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
#ifndef IMU_SENSOR_FILTERING_HPP___
#define IMU_SENSOR_FILTERING_HPP___ 

#include <filters/transfer_function.h>
#include <std_msgs/String.h>

// #include "filters/filter_base.h"
// #include "filters/realtime_circular_buffer.h" 


#include <ros/ros.h>

#include <std_msgs/Float64.h>

#include <ros/callback_queue.h>

#include <gazebo/sensors/ImuSensor.hh>
 #include <sensor_msgs/Imu.h>

#define n_data 10


namespace IMU_Sensor_Filtering
{ 


class MultiChannelTransferFunctionFilter: public filters::MultiChannelTransferFunctionFilter<double>
{
 public:
   
    void IMU_Sensor_Read(std::string filtering_imu_sensor_params_name);

    void IMU_Sensor_Filtering();

    void init(std::string filtering_imu_sensor_params_name);

    bool config();

    void cleanup();

    void pub_IMU_filtered(ros::Publisher publisher);

    //int number_of_channels_;
    std::string imu_sensor_name;
    

 private:

 	void read(const sensor_msgs::Imu::ConstPtr &imu_msg);
    
    ros::NodeHandle nh_;

    //std::string ft_sensor_name;
    std::string imu_sensor_frame_id;
    std::string imu_sensor_topic;
    //std::string filtering_ft_sensor_params_name;

    std::string filter_name;
    std::string filter_type;
    // std::string a_str;
    // std::string b_str;

    // std::vector<double> a_;   //Transfer functon coefficients (output).
    // std::vector<double> b_;   //Transfer functon coefficients (input).

    boost::shared_ptr<ros::AsyncSpinner> subscriber_spinner_;
    ros::CallbackQueue subscriber_queue_;
    
    ros::Subscriber imu_data_subscriber;
    /// \brief Ros Publisher for imu data.
    ros::Publisher imu_data_publisher;

        /// \brief Ros IMU message.
    sensor_msgs::Imu imu_msg_in;

    /// \brief Ros IMU message.
    sensor_msgs::Imu imu_msg_out;



    // double force_[3];
    // double torque_[3];
    double data[n_data];
    //double data_in[6];
    // std::vector<double> data_in;
    // std::vector<double> data_out;

    //filters::RealtimeCircularBuffer<double> input_buffer_;
    //filters::RealtimeCircularBuffer<double> output_buffer_;

};

}

#endif
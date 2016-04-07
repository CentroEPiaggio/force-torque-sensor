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
#include <IMUSensor/imu_sensor_filtering.h>
#include <std_msgs/String.h>
#include <string>  


namespace IMU_Sensor_Filtering
{ 

  void MultiChannelTransferFunctionFilter::init(std::string filtering_imu_sensor_params_name)
  { 
    nh_.getParam(filtering_imu_sensor_params_name+"/imu_sensor_name", imu_sensor_name);
    nh_.getParam(filtering_imu_sensor_params_name+"/imu_sensor_frame_id",imu_sensor_frame_id);
    ROS_INFO_STREAM("filtering_name: "<< imu_sensor_name);
    ROS_INFO_STREAM("filtering_ft_sensor_frame_id: "<< imu_sensor_frame_id);
    
    nh_.getParam("LowPass/name", filter_name);
    ROS_INFO_STREAM(" IMU filter_name: "<< filter_name);
    nh_.getParam("LowPass/type", filter_type);
    ROS_INFO_STREAM("IMU filter_type: "<< filter_type);

    this->filter_type_=filter_name;
    this->filter_type_=filter_type;
    this->number_of_channels_=n_data;

  }	

  bool MultiChannelTransferFunctionFilter::config()
{
  // Parse a and b into a std::vector<double>.
   	
  if (!nh_.getParam("LowPass/params/a", a_))
  {
    ROS_ERROR("IMU TransferFunctionFilter, \"%s\", params has no attribute a.", filter_name.c_str());
    return false;
  }
  else{  
    //ROS_INFO("config_filter_a : %f ,%f",(float)a_[0],(float)a_[1]);
    ROS_INFO("Loaded a");
  }

   if (!nh_.getParam("LowPass/params/b", b_))
   {
     ROS_ERROR("IMU TransferFunctionFilter, \"%s\", params has no attribute b.", filter_name.c_str());
     return false;
   }
   else{
   	//ROS_INFO("config_filter_b : %f ,%f",(float)b_[0],(float)b_[1]);
   ROS_INFO("Loaded IMU b");
   }
   ///\todo check length

  // // Create the input and output buffers of the correct size.
  temp_.resize(this->number_of_channels_);

  ROS_INFO("b_.size = %f",(float)b_.size());
  input_buffer_.reset(new filters::RealtimeCircularBuffer<std::vector<double> >(b_.size()-1, temp_));
  output_buffer_.reset(new filters::RealtimeCircularBuffer<std::vector<double> >(a_.size()-1, temp_));

  // Prevent divide by zero while normalizing coeffs.
   if ( a_[0] == 0)
   {
     ROS_ERROR("IMU a[0] can not equal 0.");
     return false;
   } else ROS_INFO( " IMU a_[0] = %f",(float)a_[0]);

  // Normalize the coeffs by a[0].
  if(a_[0] != 1)
  { 
  	ROS_INFO("Normalizing IMU coeff");
    for(uint32_t i = 0; i < b_.size(); i++)
    {
      b_[i] = (b_[i] / a_[0]);
    }
    for(uint32_t i = 1; i < a_.size(); i++)
    {
      a_[i] = (a_[i] / a_[0]);
    }
    a_[0] = (a_[0] / a_[0]);
  }

  return true;
};

 
  void MultiChannelTransferFunctionFilter::IMU_Sensor_Filtering()
  {
  	//ROS_INFO("force %f , %f , %f ",(float)data[0],(float)data[1],(float)data[2]);
    std::vector<double> data_in,data_out;
    //data_in.resize(this->number_of_channels_);
    data_out.resize(this->number_of_channels_);

     for (int i=0; i<n_data; ++i) {
         data_in.push_back(data[i]);
         //ROS_INFO("force %f ",(float)data[i]);
     }

     //ROS_INFO("force %f,%f,%f ",(float)data_in[0],(float)data_in[1],(float)data_in[2]);
     //ROS_INFO("torque %f,%f,%f ",(float)data_in[3],(float)data_in[4],(float)data_in[5]);

     this->update(data_in,data_out);
     //ROS_INFO("force %f,%f,%f ",(float)data_out[0],(float)data_out[1],(float)data_out[2]);
     //ROS_INFO("torque %f,%f,%f ",(float)data_out[3],(float)data_out[4],(float)data_out[5]);

    imu_msg_out.header.stamp = ros::Time::now();
    imu_msg_out.header.frame_id=imu_sensor_frame_id;
    //Guassian noise is applied to all measurements
    imu_msg_out.orientation.x = data_out[0];
    imu_msg_out.orientation.y = data_out[1]; 
    imu_msg_out.orientation.z = data_out[2];
    imu_msg_out.orientation.w = data_out[3];

    imu_msg_out.linear_acceleration.x = data_out[4];
    imu_msg_out.linear_acceleration.y = data_out[5];
    imu_msg_out.linear_acceleration.z = data_out[6];

    imu_msg_out.angular_velocity.x = data_out[7];
    imu_msg_out.angular_velocity.y = data_out[8];
    imu_msg_out.angular_velocity.z = data_out[9];

  }  



  void MultiChannelTransferFunctionFilter::read(const sensor_msgs::Imu::ConstPtr &imu_msg)
  {
    imu_msg_in= *imu_msg;
    this->data[0] = imu_msg->orientation.x;
    this->data[1] = imu_msg->orientation.y; 
    this->data[2] = imu_msg->orientation.z;
    this->data[3] = imu_msg->orientation.w;

    this->data[4] = imu_msg->linear_acceleration.x;
    this->data[5] = imu_msg->linear_acceleration.y;
    this->data[6] = imu_msg->linear_acceleration.z;

    this->data[7] = imu_msg->angular_velocity.x;
    this->data[8] = imu_msg->angular_velocity.y;
    this->data[9] = imu_msg->angular_velocity.z;

  }
   
  void MultiChannelTransferFunctionFilter::IMU_Sensor_Read(std::string filtering_imu_sensor_params_name)
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
    nh_.getParam(filtering_imu_sensor_params_name+"/imu_sensor_topic", imu_sensor_topic);
    ROS_INFO_STREAM("filtering_imu_sensor_topic: "<< imu_sensor_topic);
      // ros topic subscribtions
    ros::SubscribeOptions Sub_IMU =
        ros::SubscribeOptions::create<sensor_msgs::Imu>(
          imu_sensor_topic, 1,boost::bind(&MultiChannelTransferFunctionFilter::read, this, _1),
          ros::VoidPtr(), rosnode->getCallbackQueue());
        //&Force_Torque_Sensor_Filtering::force_torque_sensor_State

    imu_data_subscriber = rosnode->subscribe(Sub_IMU);
       
    subscriber_spinner_.reset(new ros::AsyncSpinner(1, &subscriber_queue_));
    subscriber_spinner_->start();
  }  


  void MultiChannelTransferFunctionFilter::cleanup()
   {
     subscriber_spinner_->stop();
   }

  void MultiChannelTransferFunctionFilter::pub_IMU_filtered(ros::Publisher publisher)
  { 
    imu_msg_out.orientation_covariance[0] = imu_msg_in.orientation_covariance[0];
    imu_msg_out.orientation_covariance[4] = imu_msg_in.orientation_covariance[4];
    imu_msg_out.orientation_covariance[8] = imu_msg_in.orientation_covariance[8];
    imu_msg_out.angular_velocity_covariance[0] = imu_msg_in.angular_velocity_covariance[0];
    imu_msg_out.angular_velocity_covariance[4] = imu_msg_in.angular_velocity_covariance[4];
    imu_msg_out.angular_velocity_covariance[8] = imu_msg_in.angular_velocity_covariance[8];
    imu_msg_out.linear_acceleration_covariance[0] = imu_msg_in.linear_acceleration_covariance[0];
    imu_msg_out.linear_acceleration_covariance[4] = imu_msg_in.linear_acceleration_covariance[4];
    imu_msg_out.linear_acceleration_covariance[8] = imu_msg_in.linear_acceleration_covariance[8];
  	publisher.publish(imu_msg_out);
  }
   
}




 int main(int argc, char** argv){

   try{
     ROS_INFO("starting IMU filtering node");
     ros::init(argc, argv, "IMU_sensor_filtering");

     ros::NodeHandle nh;

     std::string filtering_imu_sensor_params_name;

     nh.getParam("imu_sensor_params_name", filtering_imu_sensor_params_name);
     ROS_INFO_STREAM("Filtering_imu_sensor_params_name: "<< filtering_imu_sensor_params_name);
     
     IMU_Sensor_Filtering::MultiChannelTransferFunctionFilter filter;
     
     filter.init(filtering_imu_sensor_params_name);
     //filter.configure();  //filtering_ft_sensor_params_name+"/params/"
     filter.config();

     filter.IMU_Sensor_Read(filtering_imu_sensor_params_name);
     //Force_Torque_Sensor_Filtering::MultiChannelTransferFunctionFilter::Force_Torque_Sensor_Read data_acq(filtering_ft_sensor_params_name);
     //Force_Torque_Sensor_Filtering::MultiChannelTransferFunctionFilter::Force_Torque_Sensor_Read filter_acq(filtering_ft_sensor_params_name);
     //filter_sub.init(filtering_ft_sensor_params_name);
     //filters::SingleChannelTransferFunctionFilter filter;
     
     

     ros::NodeHandle filter_nh("IMU_filtering");

     ros::Publisher Pub_IMU_ = nh.advertise<sensor_msgs::Imu>(filter.imu_sensor_name+"/IMU_filtered",1,true);

     ros::AsyncSpinner spinner(2);
     spinner.start();

     ros::Rate loop_rate(1000);

     ros::Time last_time = ros::Time::now();

    while (ros::ok())
     {
       //ROS_INFO("force_torque_sensor_filtering is here");
       ros::Time current_time = ros::Time::now();
       ros::Duration elapsed_time = current_time - last_time;
       last_time = current_time;

       //filter.update(data_in,data_out);
       filter.IMU_Sensor_Filtering();
       filter.pub_IMU_filtered(Pub_IMU_);
       loop_rate.sleep();
     }

     filter.cleanup();
   }
   catch(...)
   {
     ROS_ERROR("Unhandled exception!");
    return -1;
   }

  return 0;
}

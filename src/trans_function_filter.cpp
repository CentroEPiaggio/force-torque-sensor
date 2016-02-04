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
#include <FTSensor/force_torque_sensor_filtering.h>
#include <std_msgs/String.h>
#include <string>  


namespace Force_Torque_Sensor_Filtering
{ 

  void MultiChannelTransferFunctionFilter::init(std::string filtering_ft_sensor_params_name)
  { 
    nh_.getParam(filtering_ft_sensor_params_name+"/ft_sensor_name", ft_sensor_name);
    nh_.getParam(filtering_ft_sensor_params_name+"/ft_sensor_frame_id",ft_sensor_frame_id);
    //ROS_INFO_STREAM("filtering_name: "<< ft_sensor_name);
    //ROS_INFO_STREAM("filtering_ft_sensor_frame_id: "<< ft_sensor_frame_id);
    
    nh_.getParam("LowPass/name", filter_name);
    ROS_INFO_STREAM("filter_name: "<< filter_name);
    nh_.getParam("LowPass/type", filter_type);
    ROS_INFO_STREAM("filter_type: "<< filter_type);

    //nh_.getParam("LowPass/params/a", a_);
    //ROS_INFO_STREAM("filter_a: "<< a_);

    //nh_.getParam("LowPass/params/b", b_);
    //ROS_INFO_STREAM("filter_b: "<< b_);

    //number_of_channels_ = 6;
    //this->number_of_channels_=6;
    this->filter_type_=filter_name;
    this->filter_type_=filter_type;
    this->number_of_channels_=6;

  }	

  bool MultiChannelTransferFunctionFilter::config()
{
  // Parse a and b into a std::vector<double>.
   	
  if (!nh_.getParam("LowPass/params/a", a_))
  {
    ROS_ERROR("TransferFunctionFilter, \"%s\", params has no attribute a.", filter_name.c_str());
    return false;
  }
  else{  
    //ROS_INFO("config_filter_a : %f ,%f",(float)a_[0],(float)a_[1]);
    ROS_INFO("Loaded a");
  }

   if (!nh_.getParam("LowPass/params/b", b_))
   {
     ROS_ERROR("TransferFunctionFilter, \"%s\", params has no attribute b.", filter_name.c_str());
     return false;
   }
   else{
   	//ROS_INFO("config_filter_b : %f ,%f",(float)b_[0],(float)b_[1]);
   ROS_INFO("Loaded b");
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
     ROS_ERROR("a[0] can not equal 0.");
     return false;
   } else ROS_INFO("a_[0] = %f",(float)a_[0]);

  // Normalize the coeffs by a[0].
  if(a_[0] != 1)
  { 
  	ROS_INFO("Normalizing coeff");
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

 
  void MultiChannelTransferFunctionFilter::Force_Torque_Sensor_Filtering()
  {
  	//ROS_INFO("force %f , %f , %f ",(float)data[0],(float)data[1],(float)data[2]);
    std::vector<double> data_in,data_out;
    //data_in.resize(this->number_of_channels_);
    data_out.resize(this->number_of_channels_);

     for (int i=0; i<6; ++i) {
         data_in.push_back(data[i]);
         //ROS_INFO("force %f ",(float)data[i]);
     }

     //ROS_INFO("force %f,%f,%f ",(float)data_in[0],(float)data_in[1],(float)data_in[2]);
     //ROS_INFO("torque %f,%f,%f ",(float)data_in[3],(float)data_in[4],(float)data_in[5]);

     this->update(data_in,data_out);
     //ROS_INFO("force %f,%f,%f ",(float)data_out[0],(float)data_out[1],(float)data_out[2]);
     //ROS_INFO("torque %f,%f,%f ",(float)data_out[3],(float)data_out[4],(float)data_out[5]);

     WrenchStamped_out.wrench.force.x = data_out[0];
     WrenchStamped_out.wrench.force.y = data_out[1];
     WrenchStamped_out.wrench.force.z = data_out[2];
     WrenchStamped_out.wrench.torque.x = data_out[3];
     WrenchStamped_out.wrench.torque.y = data_out[4];
     WrenchStamped_out.wrench.torque.z = data_out[5];

     WrenchStamped_out.header.stamp = ros::Time::now();
     WrenchStamped_out.header.frame_id = ft_sensor_frame_id;

     //Pub_Wrench_.publish(WrenchStamped_out);

  }  



  void MultiChannelTransferFunctionFilter::read(const geometry_msgs::WrenchStamped::ConstPtr &_ws)
  {
    WrenchStamped = *_ws;
     data[0] = WrenchStamped.wrench.force.x;
     data[1] = WrenchStamped.wrench.force.y;
     data[2] = WrenchStamped.wrench.force.z;
     data[3] = WrenchStamped.wrench.torque.x;
     data[4] = WrenchStamped.wrench.torque.y;
     data[5] = WrenchStamped.wrench.torque.z;
  }
   
  void MultiChannelTransferFunctionFilter::Force_Torque_Sensor_Read(std::string filtering_ft_sensor_params_name)
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
    nh_.getParam(filtering_ft_sensor_params_name+"/ft_sensor_topic", ft_sensor_topic);
    ROS_INFO_STREAM("filtering_ft_sensor_topic: "<< ft_sensor_topic);
      // ros topic subscribtions
    ros::SubscribeOptions Sub_Wrench =
        ros::SubscribeOptions::create<geometry_msgs::WrenchStamped>(
          ft_sensor_topic, 1,boost::bind(&MultiChannelTransferFunctionFilter::read, this, _1),
          ros::VoidPtr(), rosnode->getCallbackQueue());
        //&Force_Torque_Sensor_Filtering::force_torque_sensor_State

    Sub_Wrench_ = rosnode->subscribe(Sub_Wrench);
       
    subscriber_spinner_.reset(new ros::AsyncSpinner(1, &subscriber_queue_));
    subscriber_spinner_->start();
  }  


  void MultiChannelTransferFunctionFilter::cleanup()
   {
     subscriber_spinner_->stop();
   }

  void MultiChannelTransferFunctionFilter::pub_Wrench_filtered(ros::Publisher publisher)
  {
  	publisher.publish(WrenchStamped_out);
  }
   
}




 int main(int argc, char** argv){

   try{
     ROS_INFO("starting");
     ros::init(argc, argv, "force_torque_sensor_filtering");

     ros::NodeHandle nh;

     std::string filtering_ft_sensor_params_name;

     nh.getParam("ft_sensor_params_name", filtering_ft_sensor_params_name);
     ROS_INFO_STREAM("Filtering_ft_sensor_params_name: "<< filtering_ft_sensor_params_name);
     
     Force_Torque_Sensor_Filtering::MultiChannelTransferFunctionFilter filter;
     
     filter.init(filtering_ft_sensor_params_name);
     //filter.configure();  //filtering_ft_sensor_params_name+"/params/"
     filter.config();

     filter.Force_Torque_Sensor_Read(filtering_ft_sensor_params_name);
     //Force_Torque_Sensor_Filtering::MultiChannelTransferFunctionFilter::Force_Torque_Sensor_Read data_acq(filtering_ft_sensor_params_name);
     //Force_Torque_Sensor_Filtering::MultiChannelTransferFunctionFilter::Force_Torque_Sensor_Read filter_acq(filtering_ft_sensor_params_name);
     //filter_sub.init(filtering_ft_sensor_params_name);
     //filters::SingleChannelTransferFunctionFilter filter;
     
     

     ros::NodeHandle filter_nh("force_torque_sensor_filtering");

     ros::Publisher Pub_Wrench_ = nh.advertise<geometry_msgs::WrenchStamped>(filter.ft_sensor_name+"/force_torque_sensor_filtered",1,true);

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
       filter.Force_Torque_Sensor_Filtering();
       filter.pub_Wrench_filtered(Pub_Wrench_);
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

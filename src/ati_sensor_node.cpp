#include <force_torque_sensor_controller/force_torque_sensor_controller.h>
#include <ati-force-torque-sensor/ati_ft_sensor_hw.h>
#include <controller_manager/controller_manager.h>

// ROS headers
#include <ros/ros.h>
#include <std_msgs/Bool.h>

bool g_quit = false;

void quitRequested(int sig)
{
    g_quit = true;
}

int main( int argc, char** argv )
{
    
    realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> test();
  // initialize ROS
  ros::init(argc, argv, "ati_ft_sensor_hw_interface"/*, ros::init_options::NoSigintHandler*/);

  // ros spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

//   // custom signal handlers
//   signal(SIGTERM, quitRequested);
//   signal(SIGHUP, quitRequested);
//   signal(SIGINT, quitRequested);

  // create a node
  ros::NodeHandle ati_sensor_nh;
  ati_hw::ATIHW ati_sensor_hw;
  
  // get params or give default values
  std::string name, ip, frame_id, safety_topic;
  double rate;
  double wrench_threshold;
  
  ati_sensor_nh.param("name", name, std::string("ati_sensor"));
  ati_sensor_nh.param("ip", ip, std::string("192.168.1.101") );
  ati_sensor_nh.param("frame_id", frame_id, std::string("sensor_frame_id"));
  ati_sensor_nh.param("rate", rate, 500.0);
  ati_sensor_nh.setParam("publish_rate",rate);
  ati_sensor_nh.param("safety_threshold",wrench_threshold, 0.5);
  ati_sensor_nh.param("safety_topic",safety_topic, std::string("emergency_event"));
  
  ros::Publisher emergency_event_pub = ati_sensor_nh.advertise<std_msgs::Bool>(safety_topic,1);

  if(!ati_sensor_hw.init(name, frame_id, ip))
  {
    ROS_FATAL_NAMED("ati_sensor_hw","Could not initialize sensor");
    return -1;
  }

  // timer variables
  struct timespec ts = {0, 0};
  ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(1.0);

  std::cout << "Sensors names: ";
  for(auto s:ati_sensor_hw.getNames())
      std::cout << s << " ";
  std::cout << std::endl;
  //the controller manager
  force_torque_sensor_controller::ForceTorqueSensorController manager;
  manager.init(&ati_sensor_hw, ati_sensor_nh, ati_sensor_nh);

  bool started_manager(false);
  
  // run as fast as the robot interface, or as fast as possible
  ros::Rate ros_rate(rate);
  while( ros::ok() )
  {
    // get the time / period
    if (!clock_gettime(CLOCK_MONOTONIC, &ts))
    {
      now.sec = ts.tv_sec;
      now.nsec = ts.tv_nsec;
      period = now - last;
      last = now;
    } 
    else
    {
      ROS_FATAL("Failed to poll realtime clock!");
      break;
    }
    
    if(!started_manager)
    {
        manager.starting(now);
        started_manager = true;
    }

    // read the state from the sensor
    ati_sensor_hw.updateReadings();
    if(ati_sensor_hw.getNormalizeWrench() > wrench_threshold)
    {
        std_msgs::Bool emergency_event_msg;
        emergency_event_msg.data = true;
        emergency_event_pub.publish(emergency_event_msg);
    }
    
    manager.update(now, period);
    ros_rate.sleep();
  }

  spinner.stop();

  return 0;
}

#ifndef ATI_FT_SENSOR_HW_H_
#define ATI_FT_SENSOR_HW_H_

// ROS headers
#include <std_msgs/Duration.h>
#include <FTSensors.h>
#include <std_srvs/Empty.h>


// ROS controls
#include <hardware_interface/force_torque_sensor_interface.h>
#include <force_torque_sensor_controller/force_torque_sensor_controller.h>

namespace ati_hw
{

class ATIHW : public hardware_interface::ForceTorqueSensorInterface
{
public:
    
    ATIHW() {}
    virtual ~ATIHW() {}
    
//     void create(std::string name, std::string urdf_string);
    bool init(ros::NodeHandle n, std::string sensor_name, std::string frame_id, std::string ip,  bool startDataStream=false);
    void updateReadings();
    double getNormalizeWrench();
    // Strings
    std::string robot_namespace_;
    
private:
    FTSensors::ATI::NetFT ati_sensor;
    double ati_force_[3];
    double ati_torque_[3];
    double max_force[3];
    double max_torque[3];
    bool calibrate(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    ros::ServiceServer srv_calibrate;
    
}; // class

} // namespace

#endif

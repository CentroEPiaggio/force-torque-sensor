#ifndef ATI_FT_SENSOR_HW_H_
#define ATI_FT_SENSOR_HW_H_

// ROS headers
#include <std_msgs/Duration.h>
#include <FTSensors.h>

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
    bool init(std::string sensor_name, std::string frame_id, std::string ip);
    void updateReadings();
    
    // Strings
    std::string robot_namespace_;
    
private:
    FTSensors::ATI::NetFT ati_sensor;
    double ati_force_[3];
    double ati_torque_[3];
    
}; // class

} // namespace

#endif

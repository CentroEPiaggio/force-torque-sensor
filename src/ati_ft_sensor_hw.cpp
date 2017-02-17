#include <ati-force-torque-sensor/ati_ft_sensor_hw.h>

#define DEBUG 0

namespace ati_hw {

bool ATIHW::init(ros::NodeHandle n, std::string sensor_name, std::string frame_id, std::string ip, bool startDataStream)
{
    bool is_ok =  true;
    this->registerHandle(hardware_interface::ForceTorqueSensorHandle(sensor_name, frame_id, ati_force_, ati_torque_));
    is_ok &= ati_sensor.setIP(ip);
    is_ok &= ati_sensor.setFilterFrequency(FTSensors::ATI::FilterFrequency::FILTER_838_HZ);
    is_ok &= ati_sensor.setForceUnit(FTSensors::ATI::ForceUnit::N);
    is_ok &= ati_sensor.setTorqueUnit(FTSensors::ATI::TorqueUnit::Nm);
    is_ok &= ati_sensor.setDataRate(1000);
    is_ok &= ati_sensor.startDataStream(true);
    is_ok &= ati_sensor.getForceSensingRange(max_force[0], max_force[1], max_force[2]);
    is_ok &= ati_sensor.getTorqueSensingRange(max_torque[0], max_torque[1], max_torque[2]);

    srv_calibrate = n.advertiseService("calibrate",\
                           &ATIHW::calibrate, \
                           this);

    
#if DEBUG>1
    std::cout   << "Sensing Range Fx: " << max_force[0] << ". Fy: " << max_force[1] << ". Fz: "<< max_force[2]
                << " Tx: " << max_torque[0] << ". Ty: " << max_torque[1] << ". Tz: "<< max_torque[2] << std::endl;
#endif

    return is_ok;
    
}

void ATIHW::updateReadings()
{
    double fx, fy, fz, tx, ty, tz;
    if(ati_sensor.getData(fx, fy, fz, tx, ty, tz))
    {
        ati_force_[0] = fx;
        ati_force_[1] = fy;
        ati_force_[2] = fz;
        ati_torque_[0] = tx;
        ati_torque_[1] = ty;
        ati_torque_[2] = tz;
    }
}

double ATIHW::getNormalizeWrench()
{
    double norm = 0;
    norm =  std::sqrt(ati_force_[0]*ati_force_[0]/(max_force[0]*max_force[0]) + 
            ati_force_[1]*ati_force_[1]/(max_force[1]*max_force[1]) +
            ati_force_[2]*ati_force_[2]/(max_force[2]*max_force[2]) +
            ati_torque_[0]*ati_torque_[0]/(max_torque[0]*max_torque[0]) + 
            ati_torque_[1]*ati_torque_[1]/(max_torque[1]*max_torque[1]) +
            ati_torque_[2]*ati_torque_[2]/(max_torque[2]*max_torque[2]));
    return norm;            
    
}

bool ATIHW::calibrate(std_srvs::Empty::Request& req,\
                     std_srvs::Empty::Response& res)
{
    ati_sensor.calibration();
    return true;
}

}

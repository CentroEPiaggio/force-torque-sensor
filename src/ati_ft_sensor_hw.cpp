
#include <ati-force-torque-sensor/ati_ft_sensor_hw.h>

namespace ati_hw {

bool ATIHW::init(std::string sensor_name, std::string frame_id, std::string ip)
{
    bool is_ok =  true;
    this->registerHandle(hardware_interface::ForceTorqueSensorHandle(sensor_name, frame_id, ati_force_, ati_torque_));
    is_ok &= ati_sensor.setIP(ip);
    is_ok &= ati_sensor.setFilterFrequency(FTSensors::ATI::FilterFrequency::FILTER_838_HZ);
    is_ok &= ati_sensor.setForceUnit(FTSensors::ATI::ForceUnit::N);
    is_ok &= ati_sensor.setTorqueUnit(FTSensors::ATI::TorqueUnit::Nm);
    is_ok &= ati_sensor.setDataRate(1000);
    is_ok &= ati_sensor.startDataStream(true);
    
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
}

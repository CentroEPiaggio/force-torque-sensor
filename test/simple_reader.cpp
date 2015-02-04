#include <string>
#include <stdio.h> 

// FTSensor class definition
#include "FTSensor/FTSensor.h"

int main(int argc, char **argv) 
{
  // The sensor object
  FTSensor* ftsensor;

  // Create a new sensor
  ftsensor = new FTSensor();

  // ip of the FT sensor, change if necessary
  // TODO make the ip an argument
  std::string ip = "192.168.0.201";
  char* IP = new char[ip.size() + 1];
  std::copy(ip.begin(), ip.end(), IP);
  IP[ip.size()] = '\0'; // don't forget the terminating 0

  // Set ip of FT Sensor
  ftsensor->setIP( IP ); 

  // don't forget to free the char after finished using it
  delete[] IP;

  // Init FT Sensor
  ftsensor->init();

  // Set bias
  ftsensor->setBias();

  // The variable where the measurements are saved
  float measurements[6];
  
  // Loop to read and printout the values
  while(1)
  {
    ftsensor->getMeasurements(measurements);  
    cout << "Fx: " <<   measurements[0] << "Fy: " <<   measurements[1]  << "Fz: " <<   measurements[2] 
         << "Tx: " <<   measurements[3]  << "Ty: " <<   measurements[4]  << "Tz: " <<   measurements[5] 
         << endl;
  }
  return 0;
}
// standard and socket related libraries
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <fstream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>

// XML related libraries
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/encoding.h>
#include <libxml/xmlwriter.h>

using namespace std;

#define PORT 49152 
#define COMMAND 1 // 0 for check comm, doesn't send any request, 1 to ask for the values
#define NUM_SAMPLES 1 
#define PI 3.141592653589793

// Structure for the sensor response
typedef struct response_struct {
	unsigned int rdt_sequence;
	unsigned int ft_sequence;
	unsigned int status;
	int FTData[6];
} RESPONSE;


class FTSensor{
public:
  // Socket info
  char ip_[15];
  int socketHandle_;     
  struct sockaddr_in addr_;  
  struct hostent *hePtr_;

  // Sensor parameters
  double counts_per_force_ ;
  double counts_per_torque_;
  
  // Communication protocol
  RESPONSE resp_; 
  char request_[8];    
  char response_[36];

  // Constructor
  FTSensor();
  ~FTSensor();
  
  // Initialization, reading parameters from XML files, etc..
  void init();
  
  // GET functions
  // Read elements from XML file
  void  getElementNames(xmlNode * a_node);
  // Read parameters
  double getCountsperForce(){return counts_per_force_;};
  double getCountsperTorque(){return counts_per_torque_;};
  // Read sensor values
  void getMeasurements(float measurements[6]);

  // SET functions
  // Write the ip of the sensor
  void setIP(char *ip_in);
  // Set the zero of the sensor	
  void setBias();
};

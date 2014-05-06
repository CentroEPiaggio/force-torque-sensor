// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
// Also add information on how to contact you by electronic and paper mail.

// Research Center "E. Piaggio"
// This file implements the FTSensor (model 17)
// Authors: Manuel Bonilla, Carlos Rosales

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

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
// This file implements the FTSensor class usable with sensors using the ATI Netbox via xml parsing
// Authors: Manuel Bonilla, Carlos Rosales

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>

#include "FTSensor/FTSensor.h"

// Class constructor, empty for now
FTSensor::FTSensor()
{

}

FTSensor::~FTSensor()
{
  close(socketHandle_);
}

// Initialization read from XML file
void FTSensor::init()
{

  // create the socket
  socketHandle_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socketHandle_ == -1) {
      printf("failed to ini sensor");
  }

  // parse the configuration 
  xmlDoc         *doc = NULL;
  xmlNode        *root_element = NULL;
  char     Filename[100];
  sprintf(Filename, "http://%s/netftapi2.xml?index=15", ip_);
  
  doc = xmlReadFile(Filename, NULL, 0);
 
  if (doc == NULL) {
      printf("error: could not parse file %s\n", Filename);
    }
  else {
      root_element = xmlDocGetRootElement(doc);
      getElementNames(root_element);
      xmlFreeDoc(doc);
    }

  xmlCleanupParser();

  // set the socket parameters
  hePtr_ = gethostbyname(ip_);
  memcpy(&addr_.sin_addr, hePtr_->h_addr_list[0], hePtr_->h_length);
  addr_.sin_family = AF_INET;
  addr_.sin_port = htons(PORT);

  // connect
  int err = connect( socketHandle_, (struct sockaddr *)&addr_, sizeof(addr_) );
  if (err == -1) {
  printf("Error to conect with sensor");
    return;
  }

}

void FTSensor::getElementNames(xmlNode * a_node)
{
  xmlNode *cur_node = NULL;
  xmlNode *cur_node_temp = NULL;
  int i=0;
  char parameter_text[40];
  char parameter_comp[40];
  for (cur_node = a_node; cur_node; cur_node = cur_node->next) {
    if (cur_node->type == XML_ELEMENT_NODE) {
	 sprintf(parameter_comp, "%s", cur_node->name);
	 if(!strcmp(parameter_comp, "cfgcpf")){
		cur_node_temp=cur_node->children;
		sprintf(parameter_text, "%s", cur_node_temp->content);
		counts_per_force_ =(double)atoi(parameter_text);
		//counts_per_torque_ =(double)1000000.0;
		continue;
	 }
	if(!strcmp(parameter_comp, "cfgcpt")){
		cur_node_temp=cur_node->children;
		sprintf(parameter_text, "%s", cur_node_temp->content);
		counts_per_torque_ =(double)atoi(parameter_text);
		//counts_per_torque_ =(double)1000000000.0;
		continue;
	 }
    }
    getElementNames(cur_node->children);
  }
  return ;
}

void FTSensor::getMeasurements(float measurements[6])
{
  // set the standard request
  *(short*)&request_[0] = htons(0x1234); /* standard header. */
  *(short*)&request_[2] = htons(COMMAND); /* per table 9.1 in Net F/T user manual. */
  *(unsigned int*)&request_[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */

  send( socketHandle_, request_, 8, 0 );
  recv( socketHandle_, response_, 36, 0 );

  resp_.rdt_sequence = ntohl(*(unsigned int*)&response_[0]);
  resp_.ft_sequence = ntohl(*(unsigned int*)&response_[4]);
  resp_.status = ntohl(*(unsigned int*)&response_[8]);
  int i;
  for( i = 0; i < 6; i++ ) {
	  resp_.FTData[i] = ntohl(*(unsigned int*)&response_[12 + i * 4]);
	  if (i<3)
		measurements[i]=(float) resp_.FTData[i]/(float) counts_per_force_;
	  else
		measurements[i]=(float) resp_.FTData[i]/(float) counts_per_torque_;
  }
  
  return;
}

void FTSensor::setIP(char *ip_in)
{  
  sprintf(ip_, "%s",ip_in);
}

void FTSensor::setBias()
{

  char     Filename[300];

  *(short*)&request_[0] = htons(0x1234);
  *(short*)&request_[2] = htons(0x0042); 
  *(unsigned int*)&request_[4] = htonl(NUM_SAMPLES);

  hePtr_ = gethostbyname(ip_);
  memcpy(&addr_.sin_addr, hePtr_->h_addr_list[0], hePtr_->h_length);
  addr_.sin_family = AF_INET;
  addr_.sin_port = htons(PORT);

  int err = connect( socketHandle_, (struct sockaddr *)&addr_, sizeof(addr_) );
  if (err == -1) {
	printf("Error to conect with sensor");
	  return;
  }  
  send( socketHandle_, request_, 8, 0 );

  return;
}
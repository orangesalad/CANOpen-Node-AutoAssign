/*
 * Copyright (C) 2020 Ryan Higdon. All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "main.h"


#define MAX_SLAVE_NODES 127

struct LSSId slaveNodes[MAX_SLAVE_NODES];

static int canBusInit(void);

int main(void)
{

	// Init can sockets 
	int canSock = canBusInit();

	//Check if socket valid
	if(canSock < 1)
	 return canSock;

	// Slave Nodes are assumed to be started and in "PRE-OPERAITONAL" state after boot up
	// We should get some NMT Boot messages but we cannot know what nodes are there as
	// They will all have same Node ID
	canOpenInit();

	// Put all slave nodes into NMT STOPPED state. They must be in this state to use LSS
	sendNMTState(canSock, 0, STOPPED);

	// Put all slaves into LSS Wait state. In this we cannot change slave Node ID but 
	// nodes are ready to perform LSS actions 
	sendGlobalLSSState(canSock, WAIT);
	
	printf("Starting FastScan...\r\n");

	int nodesFound = 0;
	int currentScanNodes = 1;
	int i = 1;
	
	while( currentScanNodes && i < MAX_SLAVE_NODES )
	{
		currentScanNodes = fastScan(&slaveNodes[i - 1], canSock, i);
		if( currentScanNodes )
		{
			printf("Found new Node. Assigned ID: %d\n", i);
			nodesFound++;
		}
		i++; 
	}
    
	printf( "%d Nodes Found\n", nodesFound);

	for( int x = 0; x < nodesFound; x++ )
	{
		printf("Node %d:\nVendorID=0x%X\nProd ID=0x%X\nRevision=0x%X\nSN=0x%X\n", \
			x + 1, \
			slaveNodes[x].vendorId, \
			slaveNodes[x].productCode, \
			slaveNodes[x].revision, \
			slaveNodes[x].serialNum);
	}
	
	return 0;
}

static int canBusInit(void)
{
	int sock;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct ifreq ifr;
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 10000; //10ms
	

	// This should change depending on boards interface
	const char *ifname = "vcan0";

	if((sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while opening socket");
		return -1;
	}

	strcpy(ifr.ifr_name, ifname);
	ioctl(sock, SIOCGIFINDEX, &ifr);
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if(bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Error in socket bind");
		return -2;
	}

	setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

	return sock;
}
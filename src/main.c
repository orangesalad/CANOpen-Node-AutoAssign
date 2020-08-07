#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <stdint.h>
#include <stdio.h>

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
	volatile int currentScanNodes = 1;
	int i = 1;
	
	while( currentScanNodes && i < MAX_SLAVE_NODES )
	{
		currentScanNodes = fastScan(canSock, i);
		if( currentScanNodes )
		{
			printf("Found new Node\n");
			nodesFound++;
		}
		i++; 
	}
    
	// Perform algo to identify all slave nodes.
	//int nodesFound = fastScan(canSock, 1);

	printf( "%d Nodes Found\n", nodesFound);
	
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
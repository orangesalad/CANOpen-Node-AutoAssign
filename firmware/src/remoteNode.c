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

#include <errno.h>


static int canBusInit(void);

volatile uint32_t vid;
volatile uint32_t pid;
volatile uint32_t rev;
volatile uint32_t sn;

volatile uint32_t LSSId[4] = {0};

volatile uint8_t NodeId;

struct can_frame canRx;
volatile int frameValid;

// NMT States
void NMTStartup(void);
void NMTPreOperational(void);
void NMTStopped(void);
void NMTOperational(void);
void NMTResetNode(void);
void NMTResetComms(void);

//LSS States
void LSSWait(void);
void LSSConfig(void);

//fastCanStates
void FastScanVID(void);
void FastScanProdId(void);
void FastScanRev(void);
void FastScanSN(void);

// fxn pointer for current state
void (*actionState)(void);

volatile int canSock = 0;

int main(void)
{
    time_t t;
    srand((unsigned) time(&t));

    // create random LSSid for demonstration
    vid = rand(); 
    pid = rand(); 
    rev = rand(); 
    sn = rand(); 

    uint32_t lssId[4] = {vid, pid, rev, sn};
    
    struct can_frame readFrame;
    struct can_frame lssResponse;
    lssResponse.can_dlc = 8;
    uint32_t currentCheck = vid;
    
    int readBytes = 0;
    
    actionState = &NMTStartup;
    printf("Starting state machine\n");
    while(1)
    {
        (*actionState)();
        readBytes = recv(canSock, &canRx, sizeof( struct can_frame), 0);
        if( readBytes > 0 )
            frameValid = 1; 
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
	tv.tv_usec = 100000; //100ms
	

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
    printf("Opened Socket %d\n", sock);

	setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
    
    canSock = sock;

	return sock;
}

void loadValues(volatile uint32_t lss[])
{
    *lss = vid;
    lss++;
    *lss = pid;
    lss++;
    *lss = rev;
    lss++;
    *lss = sn;
    lss++;
    for( int i = 0; i < 4; i++)
        printf("0x%X\n", LSSId[i] );
    // On first boot node IS is 0xff
    NodeId = 0xff;

}

int sendBootMessage(int sock)
{
    struct can_frame bootMsg;
    bootMsg.can_id = 0x700 + NodeId;
    bootMsg.can_dlc = 1;
    bootMsg.data[0] = 0;

    int writeBytes = write(sock, &bootMsg, sizeof( struct can_frame ));
    
    if( writeBytes <  0 )
        return 0;
    
    return 1;

}

void writeToEEPROM(int id)
{

}



// NMT States
void NMTStartup(void)
{    
    //init can peripheral
    canBusInit();
    printf("SockAddress: %d\n", canSock);
    int sock  = canSock;
    //load LSS and NodeID from non-volatile
    loadValues(LSSId);
    int rtn = sendBootMessage(sock); 
    
    if( rtn )
    {
        actionState = &NMTPreOperational;
        printf("Moving to Pre-Operational State\n");
    }
    // else // go to some error
    //     actionState = error;

}
// Simply wait for new CAN message
void NMTPreOperational(void)
{

    // We recieved global or our node id NMT Message
    if( frameValid && canRx.can_id == 0 && ( canRx.data[1] == NodeId || canRx.data[1] == 0 ))
    {
        printf("Recieved NMT Message\n");
        // Check next state command
        switch( canRx.data[0] )
        {
            case 1: //Enter operational
                actionState = &NMTOperational;
                break;
            case 2: //Enter Stop
                actionState = &NMTStopped;
                printf("Moving to Stopped State\n");
                break;
            //There are more, we will ignore for now...
        }
        
        frameValid = 0;
    }
}
//In stopped, we can do LSS
void NMTStopped(void)
{
    // Recieved LSS Switch to config
    if( frameValid && canRx.can_id == 0x7E5 && ( canRx.data[0] == 4 || canRx.data[1] == 0 )) 
    {
        actionState = &LSSConfig;
        printf("Moving to LSS Config\n");
        frameValid = 0;
    }
}
void NMTOperational(void)
{
    /* Do GPIO Things... wait for CAN commands... */
}
void NMTResetNode(void);
void NMTResetComms(void);

//LSS States
void LSSWait(void)
{
    // recieved message to identify ourself if unconfigured
    if( NodeId == 0xFF && frameValid && canRx.can_id == 0x7E5 && canRx.data[0] == 4 && canRx.data[2] == 0 ) 
    {
        printf("Recieved the new message\n");
        // respond to master that we are ready to do fast can
        struct can_frame lssIdentifyResp;
        lssIdentifyResp.can_id = 0x7E4;
        lssIdentifyResp.can_dlc = 8;

        // zero data
        for( int i = 0; i < 8; i++ )
            lssIdentifyResp.data[i] = 0;


        lssIdentifyResp.data[0] = 0x4F;     

        int writeBytes = write(canSock, &lssIdentifyResp, sizeof( struct can_frame ));   
        
        if( writeBytes < 0 )
            printf("Write error in lss config state\n");
        
        frameValid = 0;
        actionState = &LSSConfig;
        printf("Moving to LSS Config\n");

    }
}
void LSSConfig(void)
{
    // recieved message to identify ourself if unconfigured
    if( NodeId == 0xFF && frameValid && canRx.can_id == 0x7E5 && canRx.data[0] == 0x81 && canRx.data[5] == 0x80 ) 
    {
        // respond to master that we are ready to do fast can
        struct can_frame lssIdentifyResp;
        lssIdentifyResp.can_id = 0x7E4;
        lssIdentifyResp.can_dlc = 8;

        // zero data
        for( int i = 0; i < 8; i++ )
            lssIdentifyResp.data[i] = 0;


        lssIdentifyResp.data[0] = 0x4F;     

        int writeBytes = write(canSock, &lssIdentifyResp, sizeof( struct can_frame ));   
        
        if( writeBytes < 0 )
            printf("Write error in lss config state\n");
        
        frameValid = 0;
        actionState = &FastScanVID;
        printf("Moving to FastCan VID\n");

    }
    // Master assigning new node ID to this slave
    if( frameValid && canRx.can_id == 0x7E5 && canRx.data[0] == 0x11)
    {
        NodeId = canRx.data[1];
        struct can_frame lssNodeIdResp;
        lssNodeIdResp.can_id = 0x7E4;
        lssNodeIdResp.can_dlc = 8;
        // zero data
        for( int i = 0; i < 8; i++ )
            lssNodeIdResp.data[i] = 0;

        lssNodeIdResp.data[0] = canRx.data[0];

        int writeBytes = write(canSock, &lssNodeIdResp, sizeof( struct can_frame ));   
        
        if( writeBytes < 0 )
            printf("Write error in lss config state\n");

        frameValid = 0;

    }
    // Master assigning new node ID to this slave
    if( frameValid && canRx.can_id == 0x7E5 && canRx.data[0] == 0x17)
    {
        writeToEEPROM(NodeId);

        struct can_frame lssStoreResp;
        lssStoreResp.can_id = 0x7E4;
        lssStoreResp.can_dlc = 8;
        // zero data
        for( int i = 0; i < 8; i++ )
            lssStoreResp.data[i] = 0;

        lssStoreResp.data[0] = canRx.data[0];

        int writeBytes = write(canSock, &lssStoreResp, sizeof( struct can_frame ));   
        
        if( writeBytes < 0 )
            printf("Write error in lss config state\n");  

        printf("New ID Saved. Moving to Pre-Operational\n");
        actionState = &NMTPreOperational;
        frameValid = 0;
    }    
}

//fastCanStates
void FastScanVID(void)
{
    // We recieved a LSS message to check VID
    if( frameValid && canRx.can_id == 0x7E5 && canRx.data[0] == 0x81 && canRx.data[6] == canRx.data[7])
    {                 
        struct can_frame lssResponse;
        lssResponse.can_id = 0x7E4;
        lssResponse.can_dlc = 8;
        for(int i =0; i < 8; i++)
            lssResponse.data[i] = 0;

        int writeBytes = 0;

        uint8_t lssSub = canRx.data[6];
        uint8_t bitCheck = canRx.data[5];
        uint32_t lssCheck = 0;

        for( int i = 1; i < 5; i++)
            lssCheck |= canRx.data[i] << (i-1)*8;
        
        uint32_t bitmask = 0;
        
        for( int i = 0; i < 32 - bitCheck; i++)
            bitmask |= (1UL << i);

        if( lssCheck == (LSSId[lssSub] & bitmask) )
            writeBytes = write(canSock, &lssResponse, sizeof( struct can_frame ));
        
        frameValid = 0;
    }
    // We are recieving last VID check, this is a test of whole VID, this determines if we cont. fastscan
    else if( frameValid && canRx.can_id == 0x7E5 && canRx.data[0] == 0x81 && canRx.data[6] != canRx.data[7])
    {
   
        struct can_frame lssResponse;
        lssResponse.can_id = 0x7E4;
        lssResponse.can_dlc = 8;
        for(int i =0; i < 8; i++)
            lssResponse.data[i] = 0;

        int writeBytes = 0;

        uint8_t lssSub = canRx.data[6];
        uint8_t bitCheck = canRx.data[5];
        uint32_t lssCheck = 0;

        for( int i = 1; i < 5; i++)
            lssCheck |= canRx.data[i] << (i-1)*8;
        
        uint32_t bitmask = 0;
        
        for( int i = 0; i < 32 - bitCheck; i++)
            bitmask |= (1UL << i);

        // We have matched lss element, ready to continue
        if( lssCheck == (LSSId[lssSub] & bitmask) )
        {
            writeBytes = write(canSock, &lssResponse, sizeof( struct can_frame ));
            actionState = &FastScanProdId;
            printf("Moving to Fastscan Prod Id\n");
        }// This is not the right lss element, go to LSS Wait
        else
        {
            printf("Moving to LSSWait\n");
            actionState = &LSSWait;
        }
        
        frameValid = 0;
        
    }
}
void FastScanProdId(void)
{
    // We recieved a LSS message to check lss element
    if( frameValid && canRx.can_id == 0x7E5 && canRx.data[0] == 0x81 && canRx.data[6] == canRx.data[7])
    {
        struct can_frame lssResponse;
        lssResponse.can_id = 0x7E4;
        lssResponse.can_dlc = 8;
        for(int i =0; i < 8; i++)
            lssResponse.data[i] = 0;

        int writeBytes = 0;

        uint8_t lssSub = canRx.data[6];
        uint8_t bitCheck = canRx.data[5];
        uint32_t lssCheck = 0;

        for( int i = 1; i < 5; i++)
            lssCheck |= canRx.data[i] << (i-1)*8;
        
        uint32_t bitmask = 0;
        
        for( int i = 0; i < 32 - bitCheck; i++)
            bitmask |= (1UL << i);

        if( lssCheck == (LSSId[lssSub] & bitmask) )
            writeBytes = write(canSock, &lssResponse, sizeof( struct can_frame ));
        
        frameValid = 0;
    }
    //We are recieving last lss element check, this is a test of whole VID, this determines if we cont. fastscan
    else if( frameValid && canRx.can_id == 0x7E5 && canRx.data[0] == 0x81 && canRx.data[6] != canRx.data[7])
    {
        struct can_frame lssResponse;
        lssResponse.can_id = 0x7E4;
        lssResponse.can_dlc = 8;
        for(int i =0; i < 8; i++)
            lssResponse.data[i] = 0;

        int writeBytes = 0;

        uint8_t lssSub = canRx.data[6];
        uint8_t bitCheck = canRx.data[5];
        uint32_t lssCheck = 0;

        for( int i = 1; i < 5; i++)
            lssCheck |= canRx.data[i] << (i-1)*8;
        
        uint32_t bitmask = 0;
        
        for( int i = 0; i < 32 - bitCheck; i++)
            bitmask |= (1UL << i);

        // We have matched lss element, ready to continue
        if( lssCheck == (LSSId[lssSub] & bitmask) )
        {
            writeBytes = write(canSock, &lssResponse, sizeof( struct can_frame ));
            actionState = &FastScanRev;
            printf("Moving to Fastscan Rev\n");
        }// This is not the right lss element, go to LSS Wait
        else
        {
            actionState = &LSSWait;
        }
        
        frameValid = 0;
    }
}
void FastScanRev(void)
{
    // We recieved a LSS message to check lss element
    if( frameValid && canRx.can_id == 0x7E5 && canRx.data[0] == 0x81 && canRx.data[6] == canRx.data[7])
    {
        struct can_frame lssResponse;
        lssResponse.can_id = 0x7E4;
        lssResponse.can_dlc = 8;
        for(int i =0; i < 8; i++)
            lssResponse.data[i] = 0;

        int writeBytes = 0;

        uint8_t lssSub = canRx.data[6];
        uint8_t bitCheck = canRx.data[5];
        uint32_t lssCheck = 0;

        for( int i = 1; i < 5; i++)
            lssCheck |= canRx.data[i] << (i-1)*8;
        
        uint32_t bitmask = 0;
        
        for( int i = 0; i < 32 - bitCheck; i++)
            bitmask |= (1UL << i);

        //printf("0x%X, 0x%x", lssCheck, (LSSId[lssSub] & bitmask));
        if( lssCheck == (LSSId[lssSub] & bitmask) )
            writeBytes = write(canSock, &lssResponse, sizeof( struct can_frame ));
        
        frameValid = 0;
    }
    //We are recieving last lss element check, this is a test of whole VID, this determines if we cont. fastscan
    else if( frameValid && canRx.can_id == 0x7E5 && canRx.data[0] == 0x81 && canRx.data[6] != canRx.data[7])
    {
        struct can_frame lssResponse;
        lssResponse.can_id = 0x7E4;
        lssResponse.can_dlc = 8;
        for(int i =0; i < 8; i++)
            lssResponse.data[i] = 0;

        int writeBytes = 0;

        uint8_t lssSub = canRx.data[6];
        uint8_t bitCheck = canRx.data[5];
        uint32_t lssCheck = 0;

        for( int i = 1; i < 5; i++)
            lssCheck |= canRx.data[i] << (i-1)*8;
        
        uint32_t bitmask = 0;
        
        for( int i = 0; i < 32 - bitCheck; i++)
            bitmask |= (1UL << i);

        // We have matched lss element, ready to continue
        if( lssCheck == (LSSId[lssSub] & bitmask) )
        {
            writeBytes = write(canSock, &lssResponse, sizeof( struct can_frame ));
            actionState = &FastScanSN;
            printf("Moving to Fastscan SN\n");
        }// This is not the right lss element, go to LSS Wait
        else
        {
            actionState = &LSSWait;
        }
        
        frameValid = 0;
    }
}
void FastScanSN(void)
{
    // We recieved a LSS message to check lss element
    if( frameValid && canRx.can_id == 0x7E5 && canRx.data[0] == 0x81 && canRx.data[6] == canRx.data[7])
    {
        struct can_frame lssResponse;
        lssResponse.can_id = 0x7E4;
        lssResponse.can_dlc = 8;
        for(int i =0; i < 8; i++)
            lssResponse.data[i] = 0;

        int writeBytes = 0;

        uint8_t lssSub = canRx.data[6];
        uint8_t bitCheck = canRx.data[5];
        uint32_t lssCheck = 0;

        for( int i = 1; i < 5; i++)
            lssCheck |= canRx.data[i] << (i-1)*8;
        
        uint32_t bitmask = 0;
        
        for( int i = 0; i < 32 - bitCheck; i++)
            bitmask |= (1UL << i);

        //printf("0x%X, 0x%x", lssCheck, (LSSId[lssSub] & bitmask));
        if( lssCheck == (LSSId[lssSub] & bitmask) )
            writeBytes = write(canSock, &lssResponse, sizeof( struct can_frame ));
        
        frameValid = 0;
    }
    //We are recieving last lss element check, this is a test of whole VID, this determines if we cont. fastscan
    else if( frameValid && canRx.can_id == 0x7E5 && canRx.data[0] == 0x81 && canRx.data[6] != canRx.data[7])
    {
        struct can_frame lssResponse;
        lssResponse.can_id = 0x7E4;
        lssResponse.can_dlc = 8;
        for(int i =0; i < 8; i++)
            lssResponse.data[i] = 0;

        int writeBytes = 0;

        uint8_t lssSub = canRx.data[6];
        uint8_t bitCheck = canRx.data[5];
        uint32_t lssCheck = 0;

        for( int i = 1; i < 5; i++)
            lssCheck |= canRx.data[i] << (i-1)*8;
        
        uint32_t bitmask = 0;
        
        for( int i = 0; i < 32 - bitCheck; i++)
            bitmask |= (1UL << i);

        // We have matched final lss element, go to LSS Config and wait for store command
        if( lssCheck == (LSSId[lssSub] & bitmask) )
        {
            writeBytes = write(canSock, &lssResponse, sizeof( struct can_frame ));
            actionState = &LSSConfig;
            printf("Moving to LSS Operational\n");
        }// This is not the right lss element, go to LSS Wait
        else
        {
            actionState = &LSSWait;
        }
        
        frameValid = 0;
    }
}
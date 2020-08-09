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
#include <time.h>


static int canBusInit(void);

volatile uint32_t vid;
volatile uint32_t pid;
volatile uint32_t rev;
volatile uint32_t sn;

volatile uint32_t LSSId[4] = {0};

volatile uint8_t NodeId;

struct can_frame canRx;
volatile int frameValid;

/* NMT States */
///State entered  on first boot. Sends boot message on bus before pre-op
void NMTStartup(void);
///After system (CANbus, GPIO...) is setup 
void NMTPreOperational(void);
///Stop most functiality. We can go to LSS from Stopped
void NMTStopped(void);
///Normal operation, read gpios, etc.
void NMTOperational(void);
///Hard reset of node
void NMTResetNode(void);
///Resets canopen interface with new Node ID/baud
void NMTResetComms(void);

/* LSS States */
///Idle LSS state, cannot change Node ID from this state
void LSSWait(void);
///LSS Configuration, only one node can be in config at a time, full access.
void LSSConfig(void);

/* FastScan States */
void FastScan(void);

///fxn pointer for current state
void (*actionState)(void);

///Global CAN interface socket for Linux
volatile int canSock = 0;

enum FastScanMessageType{
    NO_STATE_TRANSITION, 
    FINAL_LSS_PART_CHECK,
    LSS_MESSAGE_UNKNOWN
};

static enum FastScanMessageType interpretFastScanMessage(struct can_frame *rcv);
static void processLssCheck( struct can_frame *lssFrame );
static int processLssFinalCheck( struct can_frame *lssFrame );

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

    while(1)
    {
        (*actionState)();
        // Read CAN socket in non-blocking
        readBytes = recv(canSock, &canRx, sizeof( struct can_frame), 0);
        if( readBytes > 0 )
            frameValid = 1; 
    }
	return 0;
}

/// Configured for Linux, in normal operation this would setup MCU periph
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

    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

    canSock = sock;

    return sock;
}

/// This would populate from EEPROM in the MCU
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
    int x = 0;
    printf("Vendor ID: 0x%X\n", LSSId[x]);
    x++;
    printf("Product ID: 0x%X\n", LSSId[x]);
    x++;
    printf("Revision: 0x%X\n", LSSId[x]);
    x++;
    printf("Serial: 0x%X\n", LSSId[x]);   
    // On first boot node IS is 0xff. simulate for now
    NodeId = 0xff;

}

/// Sends initial boot message after setup and before pre-op state
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

///Store to EEPROM, would be some I2C or other implementation
void writeToEEPROM(int id)
{
    //Implementation of non-volatile storage...
    NodeId = id;
}


/*********************** NMT States ***********************/
void NMTStartup(void)
{    
    //init can peripheral
    canBusInit();
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
        //Invlaidate out read buffer for next state
        frameValid = 0;
    }
}


void NMTStopped(void)
{
    // Recieved LSS Switch to config
    if( frameValid && canRx.can_id == 0x7E5 && canRx.data[0] == 4 && canRx.data[1] == 0 ) 
    {
        actionState = &LSSConfig;
        printf("Moving to LSS Config\n");
        //Invlaidate out read buffer for next state
        frameValid = 0;
    }
}

void NMTOperational(void)
{
    /* Do GPIO Things... wait for CAN commands... */
}

void NMTResetNode(void)
{
    //Reset remote node
}

void NMTResetComms(void)
{
    // Do not fully reset, just reset with new baud and node id settings 
}

/*********************** LSS States ***********************/
void LSSWait(void)
{
    // recieved message to trasition into LSS Config mode
    if( NodeId == 0xFF && frameValid && canRx.can_id == 0x7E5 && canRx.data[0] == 4 && canRx.data[2] == 0 ) 
    {
        printf("Recieved the new message\n");
        // respond to master that we are ready to do fastscan
        struct can_frame lssIdentifyResp;
        lssIdentifyResp.can_id = 0x7E4;
        lssIdentifyResp.can_dlc = 8;

        // zero data
        for( int i = 0; i < 8; i++ )
            lssIdentifyResp.data[i] = 0;

        // response data
        lssIdentifyResp.data[0] = 0x4F;     

        int writeBytes = write(canSock, &lssIdentifyResp, sizeof( struct can_frame ));   
        
        if( writeBytes < 0 )
            printf("Problem with write to bus\n");
        
        //Invalidate can buffer for next state
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
        actionState = &FastScan;
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
        // Save node id to non-volatile
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

        printf("New ID %d Saved. Moving to Pre-Operational\n", NodeId);
        actionState = &NMTPreOperational;
        frameValid = 0;
    }    
}

/*********************** FastScan States ***********************/
void FastScan(void)
{
    struct can_frame lssResponse;
    lssResponse.can_id = 0x7E4;
    lssResponse.can_dlc = 8;

    //zero data
    for(int i =0; i < 8; i++)
        lssResponse.data[i] = 0;

    int writeBytes = 0;    
    
    int finalCheckMatches = 0;

    if( frameValid )
    {
        enum FastScanMessageType lssType =  interpretFastScanMessage( &canRx );

        if( lssType == NO_STATE_TRANSITION )
        {
            processLssCheck( &canRx );
        }
        else if( lssType == FINAL_LSS_PART_CHECK )
        {
            finalCheckMatches = processLssFinalCheck( &canRx );
            
            if( finalCheckMatches )
            {
                //ack to master
                writeBytes = write(canSock, &lssResponse, sizeof( struct can_frame ));
                //actionState = &FastScanProdId;
                printf("Matched Value %d\n", canRx.data[6]);
                // Matched Serial Num, final state
                if( canRx.data[6] == 3 && canRx.data[7] == 0 )
                    actionState = &LSSConfig;
            }// This is not the right lss element, go to LSS Wait
            else
            {
                printf("Moving to LSSWait\n");
                actionState = &LSSWait;
            }

        }

        frameValid = 0;
    }

}

static enum FastScanMessageType interpretFastScanMessage(struct can_frame *rcv)
{
    if( rcv->can_id == 0x7E5 && rcv->data[0] == 0x81 && rcv->data[6] == rcv->data[7])
        return NO_STATE_TRANSITION;
    else if( rcv->can_id == 0x7E5 && rcv->data[0] == 0x81 && rcv->data[6] != rcv->data[7] )
        return FINAL_LSS_PART_CHECK;
    else
        return LSS_MESSAGE_UNKNOWN;    
}

static void processLssCheck( struct can_frame *lssFrame )
{
    struct can_frame lssResponse;
    lssResponse.can_id = 0x7E4;
    lssResponse.can_dlc = 8;

    //zero data
    for(int i =0; i < 8; i++)
        lssResponse.data[i] = 0;

    int writeBytes = 0; 

    // get currrent and next lss state checks
    uint8_t lssSub = lssFrame->data[6];
    uint8_t bitCheck = lssFrame->data[5];
    uint32_t lssCheck = 0;

    // copy the lss partial val we are checking from master
    for( int i = 4; i > 0; i--)
        lssCheck |= lssFrame->data[i] << (i-1)*8;
    
    uint32_t bitmask = 0;
    
    // Construct bitmask to compare with our lss
    for( int i = 0; i < 32 - bitCheck; i++)
        bitmask |= (1UL << (31 - i));

    // compare lss part and send ack if we have the same value
    if( lssCheck == (LSSId[lssSub] & bitmask) )
        writeBytes = write(canSock, &lssResponse, sizeof( struct can_frame ));
    
}

static int processLssFinalCheck( struct can_frame *lssFrame )
{
    int moveToNextState = 0;

    struct can_frame lssResponse;
    lssResponse.can_id = 0x7E4;
    lssResponse.can_dlc = 8;

    //zero data
    for(int i =0; i < 8; i++)
        lssResponse.data[i] = 0;

    int writeBytes = 0; 

    // get currrent and next lss state checks
    uint8_t lssSub = lssFrame->data[6];
    uint8_t bitCheck = lssFrame->data[5];
    uint32_t lssCheck = 0;

    // copy the lss partial val we are checking from master
    for( int i = 4; i > 0; i--)
        lssCheck |= lssFrame->data[i] << (i-1)*8;
    
    uint32_t bitmask = 0;
    
    // Construct bitmask to compare with our lss
    for( int i = 0; i < 32 - bitCheck; i++)
        bitmask |= (1UL << (31 - i));

    // We have matched final lss element, go to LSS Config and wait for store command
    if( lssCheck == (LSSId[lssSub] & bitmask) )
    {
        writeBytes = write(canSock, &lssResponse, sizeof( struct can_frame ));
        moveToNextState = 1;
        //actionState = &LSSConfig;
        //printf("Moving to LSS Operational\n");
    }// This is not the right lss element, go to LSS Wait

    return moveToNextState;
}
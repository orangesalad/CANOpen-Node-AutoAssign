#include "CANOpen_LSS.h"


// LSSSub/Next defines
#define VENDOR_ID   0
#define PROD_CODE   1
#define REVISION    2
#define SERIAL      3

// LSS Master Frame Indicies
#define  CS         0
#define  ID_LSB     1
#define  ID_1       2
#define  ID_2       3
#define  ID_MSB     4
#define  BITCHECK   5
#define  LSS_SUB    6
#define  LSS_NEXT   7


static uint16_t LSSMasterMessageId = 0x7E5; 

void sendGlobalLSSState(enum LSSState state)
{
    
}

int fastScan(int canSock, uint32_t id)
{
    struct can_frame readFrame;
    // Frame structure: B0:0x81, B1-4:ID#, B5:BitMask
    //                  B6:LSSSub, B7:LSSNext 
    struct can_frame lssMstrMsg;
    lssMstrMsg.can_id = LSSMasterMessageId;
    lssMstrMsg.can_dlc = 8;
    lssMstrMsg.data[CS] = 0x81;

    // Zero byte 1-7 for identify
    for( int i = 1; i < LSS_NEXT + 1; i++ )
        lssMstrMsg.data[i] = 0;

    // Set bitCheck for all node identify
    lssMstrMsg.data[BITCHECK] = 0x80;

    //Send initial Slave Identify msg
    int writeBytes = write(canSock, &lssMstrMsg, sizeof( struct can_frame ));
    // wait 10ms for response from slaves
    int readBytes = recv(canSock, &readFrame, sizeof( struct can_frame), 0);
    
    // Check response, if we timed out leave
    if( readBytes < 0 )
        return 0;

    





}
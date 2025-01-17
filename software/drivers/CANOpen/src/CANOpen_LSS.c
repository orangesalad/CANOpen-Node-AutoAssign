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

enum LSS_PART{
    VEND_ID,
    PRODUCT_CODE,
    REV,
    SN
};

static uint16_t LSSMasterMessageId = 0x7E5; 

static uint32_t findLSSPart(int sock, struct LSSId *lssid);
static int sendGlobalLSSIdentify(int sock);

void sendGlobalLSSState(int sock, enum LSSState state)
{
    struct can_frame f;
    
    f.can_id = LSSMasterMessageId;
    f.can_dlc = 8;
    // zero data
    for( int i = 0; i < 8; i++)
        f.data[i] = 0;
    

    f.data[0] = 4;

    if( state == WAIT )
        f.data[1] = 0;
    else //Config mode
        f.data[1] = 1;

    int writeBytes = write(sock, &f, sizeof( struct can_frame ));


}

int fastScan(struct LSSId *_id, int canSock, uint8_t nodeId)
{
    struct can_frame readFrame;
    // Frame structure: B0:0x81, B1-4:ID#, B5:BitMask
    //                  B6:LSSSub, B7:LSSNext 
    struct can_frame lssMstrMsg;
    lssMstrMsg.can_id = LSSMasterMessageId;
    lssMstrMsg.can_dlc = 8;
    int writeBytes = 0;
    int readBytes = 0;
    
    // Send lss identify and check for response
    if( sendGlobalLSSIdentify(canSock) ) 
    {
        struct LSSId id;
        uint8_t lssSub = 0;
        uint8_t lssNext = 0;

        id.vendorId = 0;
        id.productCode = 0;
        id.revision = 0;
        id.serialNum = 0;

        // Scan for LSS Id
        findLSSPart(canSock, &id);

        // clear can frame
        for( int i = 0; i < 8; i++)
            lssMstrMsg.data[i] = 0;

        //slave node now in lss config mode ready to accpt new node ID
        //CS is 0x11 for assign node ID
        lssMstrMsg.data[CS] = 0x11;
        lssMstrMsg.data[1] = nodeId;
        //Send update Node ID Message
        writeBytes = write(canSock, &lssMstrMsg, sizeof( struct can_frame ));
        //Wait 10ms for response
        readBytes = recv(canSock, &readFrame, sizeof( struct can_frame), 0);   
        if( readBytes < 0 )
            return 0;

        //Node ID Assign Successful
        if( readFrame.data[0] == 0x11 && readFrame.data[1] == 0)
        {
            // store ID on remote node in non-volatile, CS = 0x17
            lssMstrMsg.data[CS] = 0x17;
            lssMstrMsg.data[1] = 0;

            //Send store
            writeBytes = write(canSock, &lssMstrMsg, sizeof( struct can_frame ));
            //Wait 10ms for response
            readBytes = recv(canSock, &readFrame, sizeof( struct can_frame), 0);   
            // timeout
            if( readBytes < 0 )
                return 0;            
            // Store was successful
            if( readFrame.data[0] == 0x17 && readFrame.data[1] == 0)
            {
                // set all lss nodes back to operational
                lssMstrMsg.data[CS] = 0x4;
                //Send store
                writeBytes = write(canSock, &lssMstrMsg, sizeof( struct can_frame ));
                
                _id->vendorId = id.vendorId;
                _id->productCode = id.productCode;
                _id->revision = id.revision;
                _id->serialNum = id.serialNum; 
                
                return 1;
            }
        } 
    }

    return 0;
}


static int sendGlobalLSSIdentify(int sock)
{
    struct can_frame lssGlobalIdent;
    struct can_frame readFrame;

    for( int i = 0; i < 7; i++ )
        lssGlobalIdent.data[i] = 0;

    lssGlobalIdent.can_id = LSSMasterMessageId;
    lssGlobalIdent.can_dlc = 8;
    lssGlobalIdent.data[CS] = 0x81;
    lssGlobalIdent.data[BITCHECK] = 0x80;

    //Send initial Slave Identify msg
    int writeBytes = write(sock, &lssGlobalIdent, sizeof( struct can_frame ));
    // wait 10ms for response from slaves
    int readBytes = 1;
    int i = 0;
    int nodeFound = 0;
    // read bus but also make sure we flush extra messages
    while(readBytes > 0 && i < 127)
    {
        readBytes = recv(sock, &readFrame, sizeof( struct can_frame), 0);
        if( readBytes > 0 )
            nodeFound = 1;
        i++;
    } 
    

    // CAN Read timed out after 10ms, no remotes found
    if( !nodeFound )
        return 0;
    else if( readFrame.can_id == 0x7E4 && readFrame.data[0] == 0x4F) 
        return 1;

    return 0;

}

static uint32_t findLSSPart(int sock, struct LSSId *lssid)
{
    struct can_frame tx;
    struct can_frame rx;
    int writeBytes = 0;
    int readBytes = 0;
    uint32_t part = 0;

    uint8_t lssSub = 0;
    uint8_t lssNext = 0;

    int x = 0;
    int nodeFound = 0;

    for(int n = 0; n < 4; n++)
    {   
        part = 0;
        tx.can_id = LSSMasterMessageId;
        tx.can_dlc = 8;
        
        //clear data 
        for( int i = 0; i < 8; i++ )
            tx.data[i] = 0;

        tx.data[CS] = 0x81;
        tx.data[LSS_SUB] = lssSub;

        // iterate over bits of lss part
        for( int i = 31; i >= 0; i-- )
        {
            tx.data[LSS_NEXT] = lssNext;

            for( int j = 1; j < 5; j++ )
                tx.data[j] = (uint8_t)( 0xFF & ( (part) >> (j - 1)*8) );
            
            tx.data[BITCHECK] = i;
            //Write LSS Identify msg
            writeBytes = write(sock, &tx, sizeof( struct can_frame ));
            //Wait 10ms for response
            readBytes = 1;
            x = 0;
            nodeFound = 0;
            // read bus but also make sure we flush extra messages
            while(readBytes > 0 && x < 127)
            {
                readBytes = recv(sock, &rx, sizeof( struct can_frame), 0);
                if( readBytes > 0 )
                    nodeFound = 1;
                x++;
            } 
            // If we don't get response we know current Bitcheck bit is 1
            if( !nodeFound )
            {
                (part) |= (1UL << i); 
            }
        }

        // Change LSS Check indicies
        if( lssSub == 3 && lssNext == 3)
            lssNext = 0;
        else
            lssNext++;  

        tx.data[LSS_NEXT] = lssNext;
        // Add LSS data for final check
        for( int j = 1; j < 5; j++ )
            tx.data[j] = (uint8_t)( 0xFF & ( (part) >> (j - 1)*8) );
        
        //Check all bits
        tx.data[BITCHECK] = 0;
        //Write LSS Identify msg
        writeBytes = write(sock, &tx, sizeof( struct can_frame ));
        //Wait 10ms for response
        readBytes = 1;
        x = 0;
        nodeFound = 0;
        // read bus but also make sure we flush extra messages
        while(readBytes > 0 && x < 127)
        {
            readBytes = recv(sock, &rx, sizeof( struct can_frame), 0);
            if( readBytes > 0 )
                nodeFound = 1;
            x++;
        }
        // Move onto next LSS part
        lssSub++;
        //Assign found value to correct LSS part
        if( n == 0 )
            lssid->vendorId = part;
        else if( n == 1)
            lssid->productCode = part;
        else if( n == 2)
            lssid->revision = part;
        else if( n == 3)
            lssid->serialNum = part;
    }
    
    return 0;
}

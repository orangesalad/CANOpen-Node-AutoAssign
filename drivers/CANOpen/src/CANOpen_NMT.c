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

#include "CANOpen_NMT.h"


void sendNMTState(int sock, unsigned int nodeID, enum NMTState state)
{
    
    struct can_frame NMTStateMessage;

    // COB ID = 0 for NMT commands
    NMTStateMessage.can_id = 0;
    NMTStateMessage.can_dlc = 2;
    // Assign node ID field, 0 for all nodes
    NMTStateMessage.data[1] = nodeID;

    // Set state field
    switch(state)
    {
        case PRE_OPERATIONAL:
            NMTStateMessage.data[0] = 0x80;
            break;

        case STOPPED:
            NMTStateMessage.data[0] = 2;
            break;

        case OPERATIONAL:
            NMTStateMessage.data[0] = 1;
            break; 
        
        case RESET:
            NMTStateMessage.data[0] = 0x81;
            break;
        
        default: // Nothing
            break;
    }

    int writeBytes = write(sock, &NMTStateMessage, sizeof( struct can_frame ));
}
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
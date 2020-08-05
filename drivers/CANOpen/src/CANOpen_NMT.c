#include "CANOpen_NMT.h"


void sendNMTState(unsigned int nodeID, enum NMTState state)
{
    
    struct can_frame NMTStateMessage;

    // COB ID = 0 for NMT commands
    NMTStateMessage.can_id = 0;
    NMTStateMessage.can_dlc = 2;
    // Assign node ID field, 0 for all nodes
    NMTStateMessage.data[1] = 0;

    // Set state field
    switch(state)
    {
        case PRE_OPERATIONAL:
            NMTStateMessage.data[1] = 0x80;
            break;

        case STOPPED:
            NMTStateMessage.data[1] = 2;
            break;

        case OPERATIONAL:
            NMTStateMessage.data[1] = 1;
            break; 
        
        case RESET:
            NMTStateMessage.data[1] = 0x81;
            break;
        
        default: // Nothing
            break;
    }


}
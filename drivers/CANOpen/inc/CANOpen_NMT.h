#ifndef __CANOPEN_NMT_H__
#define __CANOPEN_NMT_H__

#include <linux/can.h>
#include <unistd.h>
#include <assert.h>

enum NMTState{
    PRE_OPERATIONAL,
    OPERATIONAL,
    STOPPED,
    RESET
};


void sendNMTState(int sock, unsigned int nodeID, enum NMTState state);

#endif


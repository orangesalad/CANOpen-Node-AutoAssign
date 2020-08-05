#ifndef __CANOPEN_LSS_H__
#define __CANOPEN_LSS_H__

#include <stdint.h>
#include <linux/can.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>

enum LSSState{
    WAIT,
    CONFIGURATION
};

struct LSSId {

    uint32_t vendorId;
    uint32_t productCode;
    uint32_t revision;
    uint32_t serialNum;

};

void sendGlobalLSSState(enum LSSState state);

int fastScan(int canSock, uint32_t id);

#endif
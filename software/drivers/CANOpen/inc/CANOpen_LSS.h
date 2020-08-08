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

#ifndef __CANOPEN_LSS_H__
#define __CANOPEN_LSS_H__

#include <stdint.h>
#include <linux/can.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>

/**
 * @brief Layer Setting Services States
 * 
 */
enum LSSState{
    WAIT,
    CONFIGURATION
};

/**
 * @brief 128-Bit device unique Layer Setting Service ID
 * 
 */
struct LSSId {

    uint32_t vendorId;
    uint32_t productCode;
    uint32_t revision;
    uint32_t serialNum;

};

/**
 * @brief Change the LSS State of all devices, can only 
 *        be used when device is in NMT Stopped State
 * 
 * @param sock CAN Socket
 * @param state Desired state to enter
 */
void sendGlobalLSSState(int sock, enum LSSState state);

/**
 * @brief Perform LSS Fastscan to identify 1 unconfigured  
 *        node on the bus
 * 
 * @param id LSSId struct wheich the found params will be copied
 * @param canSock CAN Socket
 * @param nodeId Node ID we wish to assign the found Node
 * @return int 1 if node is found, 0 otherwise
 */
int fastScan(struct LSSId *id, int canSock, uint8_t nodeId);

#endif
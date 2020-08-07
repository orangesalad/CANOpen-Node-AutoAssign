# CANOpen Auto Adressing Software

## This software is meant to implement the LSS FastScan algorithm, as part of the CANOpen standard

## Introduction
The Fastscan implemenetation is a way to detect remote nodes on a bus. It takes advantage of the fact that every remote node should have a globally unique LSS ID which can be used to identify it on the bus. 

## Environment
The software is designed to be run on a linux target. At the moment the Makefile uses GCC, so if you have that you can build the software using `make`. The binary will be placed in the `/build` directory. You will also need to have a working CAN interface to run the software. If you are on Linux, you can create a virtual CAN interface by doing the following: 

```
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

If you type `ifconfig` you should see the virtual CAN interface. You may also want to install can-utils to view the traffic on the interface.

## Demonstration
To demo the software, you can use the file in the `firmware/` directory. This file compiles for a Linux target, using the Linux CAN syscalls. In normal operation, this file would be refactored to run on a remote node which would be an MCU. You can build the firmware using `make` in the firmware directory. 

The firmware binary can be run many times simultaneously as it uses `rand()` to generate different LSS Id's for each process. With many 'nodes' running, you can then execute the Auto Adressing software and observe the remote nodes being detected. 

# Possible Improvements
1. Instead of scanning randomly, it would be best to store all previously found Vendor ID's and Product codes locally in a file, as it is probable that these types of devices would be the most to added into a system. These values would be scanned for first, and any unkown values after if needed. 

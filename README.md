# CANOpen Auto Adressing Software

## This software is meant to implement the LSS FastScan algorithm, as part of the CANOpen standard

### Introduction
The Fastscan implemenetation is a way to detect remote nodes on a bus. It takes advantage of the fact that every remote node should have a globally unique LSS ID which can be used to identify it on the bus. 

## Environment
This software is designed to be run on a linux target. At the moment the Makefile uses GCC, so if you have that you can build the software using `make`. The binary will be placed in the `/build` directory. You will also need to have a working CAN interface to run the software. If you are on Linux, you can do the following: 

```
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

Now if you run `ifconfig` you should see the virtual CAN interface. 

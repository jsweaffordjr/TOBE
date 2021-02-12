# TOBE

These MATLAB scripts and Simulink models were developed to control an 18-DOF robot built from
the BIOLOID Type-A kit, using Dynamixel AX-12+ motors. The read_write2.m script is a modification
of the read_write script supplied by ROBOTIS, and allows the user to control two motors, instead of
one. That script was the basis for the TOBEjointctrl.m script, which allows for the control of all 18
motors manually. 

To use these scripts, download the DynamixelSDK folder from Robotis's website: 
https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/#repository

Robotis Customer Support has produced YouTube videos that are helpful in getting started:
Dynamixel SDK/MATLAB software prep: https://www.youtube.com/watch?v=RcnVYqhGYiQ

Read Write Single Port control of Dynamixels using MATLAB: https://www.youtube.com/watch?v=znlH9_rIfsA

Read Write Multi Port control of Dynamixels using MATLAB: https://www.youtube.com/watch?v=eGG-RIVKtrs&t=1s
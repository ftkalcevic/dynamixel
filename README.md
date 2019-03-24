# dynamixel
Interface linuxcnc to dynamixel for robot arm

This is an incomplete dynamixel component for linuxcnc.
It is hard coded for 2 dynamixels with ids 2 and 3.  
Linuxcnc controls it using its position and velocity, and enables it using enable_torque.
It is a user component, that runs continuously.  At 3MB, it updates at about 1kHz.

There is a simple linuxcnc config in hal_test which maps the dynamixels to X and Y.

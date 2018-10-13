#er_gripper
This is a simple servo controller through arduino.

##reqs
requires an arduino, duh. also rosserial.
```bash
sudo apt-get install ros-kinetic-rosserial
```

for the arduino, I got rosserial_arduino libs for it so it's all cool. I will be uploading the code through the arduino IDE, it checks out nicely.

##topic
publish to topic `angle_msg`
arduino will need to be connected through rosserial and on correct USB port - I'm working on it, will probably just require some small additions to the launch file

#WIP
##ignore the stuff below (just my notes here)
rosrun rosserial_python serial_node.py /dev/ttyUSB0

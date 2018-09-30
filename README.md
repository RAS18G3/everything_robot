# everything_robot
Metapackage including all required packages and code to run the robot

## How to run on your own computer
Assuming you have ROS Kinetic Kame up and running, do the following to get the required RAS packages:
```console
cd
git clone https://github.com/KTH-RAS/ras_install.git
cd ras_install/scripts
./install.sh
```

At this point you should be able to run all the test commands from canvas.

Clone and compile this repository using:
```console
cd ~/catkin_ws
git clone https://github.com/RAS18G3/everything_robot.git
catkin_make
```

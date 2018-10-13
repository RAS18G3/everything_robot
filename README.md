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
cd ~/catkin_ws/src
git clone https://github.com/RAS18G3/everything_robot.git
catkin_make
```
## How to run on the robot

- Turn on the NUC on the robot

- After a while robot_group3 wifi network should appear, connect to it

- Connect via SSH to the robot via (password is ras2018)
```console
ssh ras23@10.42.0.1
```

- if you use terminator it might be better to set up a profile to automatically connect to ssh whenever the terminal is split

  - inside .config/terminator/config add this below the default profile
  ```
  [[remotehost]]
    use_custom_command = True
    exit_action = restart 
    custom_command = sshpass -p 'ras2018' ssh ras23@10.42.0.1
  ```
  
  - use the profile by runnning
  ```console
  terminator -p remotehost
  ```
  
  - you need sshpass for this, install with
  ```console
  sudo apt-get install sshpass
  ```
  
- on the robot execute
```console
roslaunch er_launch robot.launch
```

- to see messages and communicate over ROS you need to set some environment variables
```console
export ROS_MASTER_URI=http://10.42.0.1:11311
export ROS_IP=your_ip_inside_the_robot_network
```

- after setting these `rostopic list` should show the topics published on the robot

- run `rviz` on your local machine to visualize the robot, and run `rosrun er_wheels er_keyboard_node.py` on your machine to control the robot using WASD

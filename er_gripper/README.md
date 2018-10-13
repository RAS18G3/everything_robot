# er_gripper
This is a simple servo controller through arduino.

## reqs
Requires an arduino, duh. Also rosserial, not sure if it's installed by default. Otherwise:
```bash
sudo apt-get install ros-kinetic-rosserial
```

For the arduino, I got `rosserial_arduino` libs for it so it's all cool. I will be uploading the code through the arduino IDE, it checks out nicely so far.

## topic
Publish servo angle to topic `grip`.
Arduino will need to be connected through rosserial and on correct USB port - the launch file should take care of that.

## testing
If you really want to test, do the following:
1. Install Arduino IDE from https://www.arduino.cc/en/Main/Software and `rosserial_arduino`+`rosserial` as per http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
2. Connect the Arduino to your PC via USB
3. Open `arduino_code/gripper.ino` with the IDE
4. Choose board (I think it's a `Pro Mini 5V w/ ATmega328`), port (probably `ttyUSB0`) and programmer (`USBTinyISP`) and upload
5. Connect yellow cable on servo to pin 9 on Arduino, red to VCC, black to GND (protip: might be a good idea to disconnect USB while you're doing this. I hotswap constantly, but burned an LED display once because of it)
6. Make sure the port in `gripper.launch` is correct for your case. I should make it autodetect instead
6. `roslaunch er_gripper gripper.launch` - the servo should go from 0 to 180 in 5deg steps, then back


# WIP
ignore this, just some notes
rosrun rosserial_python serial_node.py /dev/ttyUSB0

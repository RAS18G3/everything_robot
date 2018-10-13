/*
  * This code is to be uploaded to the arduino,
  * preferably using the arduino IDE.
  * ros.h and std_msg/Int64.h need to be in the same dir
*/

#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int64.h>


Servo grip_servo;  // create a servo object

const int servo_pin = 9;
const int sleep_time = 100;  // loop sleep time in ms
const int angle_bounds[2] = {0, 90};

int target_angle = angle_bounds[0]; // variable to store target position for servos

void grip_callback( const std_msgs::Int64& angle_msg )
{
  int target_angle = angle_msg.data; // unpack msg

  // fix bounds
  if(target_angle < angle_bounds[0]){ target_angle = angle_bounds[0]; }
  if(target_angle > angle_bounds[1]){ target_angle = angle_bounds[1]; }

  grip_servo.write(target_angle); // write to servo
}

// arduino/node setup

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int64> sub("grip", grip_callback);

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  grip_servo.attach(servo_pin);  // attaches the servo object to the given pin
  grip_servo.write(angle_bounds[0]); // initialize servo to low bound
}

// arduino mainloop
// has no intrinsic freq
void loop()
{
  nh.spinOnce();
  delay(sleep_time);
}

/*
  * This code is to be uploaded to the arduino,
  * preferably using the arduino IDE.
  * ros.h and std_msg/Int64.h need to be in the same dir
*/

#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int64.h>

ros::NodeHandle n;
ros::Subscriber twist_sub = n.subscribe(node_name + "/twist", 1, twistCallback);
Servo gripServo;  // create a servo object

const int servoPin = 9;
const int delayTime = 100;  // loop sleep time in ms
const int angleBounds[2] = {0, 90}

int targetAngle = 0; // variable to store target position for servos

void gripCallback( const std_msgs::Int64& angle_msg )
{
  int targetAngle = angle_msg.data;
  // fix bounds
  if(targetAngle < angleBounds[0])
  {
    targetAngle = angleBounds[0];
  }
  if(targetAngle > angleBounds[1])
  {
    targetAngle = angleBounds[1];
  }

  gripServo.write(pos);
}


void setup() {
  Serial.begin(9600); // initialize serial
  gripServo.attach(servoPin);  // attaches the servo object to the given pin
  gripServo.write(target_angles[0]); // initialize servo to 0deg
}

void loop() {
  if (Serial.available() > 0) {
    // read in new target values
    read_targets();
    // send them to the servos
    servos[0].write(target_angles[0]);
    servos[1].write(target_angles[1]);
  }
  delay(sleep_time);
}



void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  gripServo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop()
{
  n.spinOnce();
  delay(delayTime);
}

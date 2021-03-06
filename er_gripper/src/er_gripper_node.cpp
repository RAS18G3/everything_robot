/*
  * only for testing pub to arduino, not necessary otherwise
*/

#include "ros/ros.h"
#include "std_msgs/Int64.h"

// add code here to publish to gripper
ros::Publisher grip_pub;
std_msgs::Int64 angle_msg;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_gripper_node");
  ros::NodeHandle n;
  grip_pub = n.advertise<std_msgs::Int64>("grip", 1);

  angle_msg.data = 0;
  int deg_delta = 1; // servo change each step

  ros::Rate loop_rate(50); // 5Hz should be slow enough for testing
  while (ros::ok())
  {
    ros::spinOnce();

    // respect the bounds and "bounce" around them...
    if(angle_msg.data > 180 | angle_msg.data < 0){ deg_delta *= -1; }

    angle_msg.data += deg_delta;
    grip_pub.publish(angle_msg);

    loop_rate.sleep();
  }

  return 0;
}

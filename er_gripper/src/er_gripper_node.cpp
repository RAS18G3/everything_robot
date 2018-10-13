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
  ros::Rate loop_rate(5); // 5Hz should be slow enough for testing

  angle_msg.data = 0;

  while (ros::ok())
  {
    ros::spinOnce();

    angle_msg.data += 5;
    grip_pub.publish(angle_msg);

    loop_rate.sleep();
  }

  return 0;
}

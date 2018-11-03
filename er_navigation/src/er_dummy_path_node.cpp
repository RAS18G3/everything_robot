#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Int64.h"

// Dummy node to publish a simple path when prodded from /pathdummy
// Only used for debug
ros::Subscriber path_sub;
ros::Publisher path_pub;

void pathdummy_callback(const std_msgs::Int64)
{
  // the message is not important at all, just callback
  nav_msgs::Path path;
  std::vector<geometry_msgs::PoseStamped> poses(2);
  poses[0].pose.position.x = 10;
  poses[0].pose.position.y = 10;
  poses[1].pose.position.x = 100;
  poses[1].pose.position.y = 100;
  path.poses = poses;
  path_pub.publish(path);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_dummy_path_node");
  ros::NodeHandle n;
  path_sub = n.subscribe("/pathdummy", 1, pathdummy_callback);
  path_pub = n.advertise<nav_msgs::Path>("/path1", 1);
  ros::spin();
  return 0;
}

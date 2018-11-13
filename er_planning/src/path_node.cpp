#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <er_planning/PathAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Float64MultiArray.h"

/*EXAMPLE HOW TO PUBLISH A PATH IN TERMINAL:
rostopic pub /path_msg std_msgs/Float64MultiArray "layout: dim:
  - label: ''
    size: 1234
    stride: 1
  data_offset: 0
data: [0.0, 0.0, 1.0, 1.0]" */

// typedef actionlib::SimpleActionClient<er_planning::PathAction> Client;
bool stop;
//
// void path_callback(const std_msgs::Float64MultiArray::ConstPtr msg){
//   Client client("path", true);
//   client.waitForServer();
//   er_planning::PathGoal goal_path;
//   std::vector<float> x;
//   for(int i = 0; i < msg->data.size(); i++){
//     x.push_back(msg->data[i]);
//   }
//   goal_path.Path = x;
//   client.sendGoal(goal_path);
//   client.waitForResult(ros::Duration(5.0));
//   if(stop){
//     client.cancelGoal();
//   }
// }
//
int main(int argc, char **argv)
{
  stop = false;
  ros::init(argc, argv, "path_node");

  ros::NodeHandle n;
  // ros::Subscriber path_sub = n.subscribe("path_msg", 10, path_callback);
  ros::Rate loop_rate(10);
  ros::spin();
}

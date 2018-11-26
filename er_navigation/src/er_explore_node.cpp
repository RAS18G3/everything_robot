#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetPlan.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <actionlib/client/simple_action_client.h>
#include "er_planning/PathGoal.h"
#include "er_planning/PathAction.h"
#include "er_planning/PathActionGoal.h"

struct PositionVector
{
  double PosX;
  double PosY;
};
struct PositionVector_int
{
  int PosX;
  int PosY;
};

ros::Duration timeout(3.0);

ros::ServiceServer explore_srv;

bool DONE;
int x_grid;
int y_grid;
double pos_x;
double pos_y;
double mapped_radius;
int width;
int height;
double resolution;
double offsetX;
double offsetY;
nav_msgs::OccupancyGrid map_msg;
std::vector<int8_t> map;
std::vector<int8_t> fog_map;

void goto_point(double x, double y){
 //find path
 ROS_INFO("going to point");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<nav_msgs::GetPlan>("/pathfinder/find_path");
  nav_msgs::GetPlan srv;
  srv.request.start.pose.position.x = pos_x;
  srv.request.start.pose.position.y = pos_y;
  srv.request.goal.pose.position.x = x;
  srv.request.goal.pose.position.y = y;

  ROS_INFO("pos_x %f", pos_x);
  ROS_INFO("pos_y %f", pos_y);
  ROS_INFO("x %f", x);
  ROS_INFO("y %f", y);

  client.call(srv);
  ROS_INFO("got plan");

  nav_msgs::Path plan;
  plan = srv.response.plan;


 //execute plan
 actionlib::SimpleActionClient<er_planning::PathAction> ac("path", true);
 ROS_INFO("waiting for server");

 ac.waitForServer();
 ROS_INFO("done waiting");

 er_planning::PathActionGoal goal;
 goal.goal.Path = plan;
 ROS_INFO("sending goal..");
 ac.sendGoal(goal.goal);
 ROS_INFO("sent goal");

  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
  ROS_INFO("Im done waiting for result");

if (finished_before_timeout)
{
  actionlib::SimpleClientGoalState state = ac.getState();
  ROS_INFO("Action finished: %s",state.toString().c_str());
}
else{
  ROS_INFO("Action did not finish before the time out.");
}
}

int pos2index(int xg, int yg){
  int index = yg*width+xg;
  return index;
}

PositionVector_int index2pos(int index){
  int i = 0;
  while(index-((i+1)*width) > 0){
    i++;
  }
  int y = i;
  int x = index-(i*width);
  PositionVector_int pos;
  pos.PosX = x;
  pos.PosY = y;
  return pos;
}
PositionVector grid_pos2coordinates(int xg, int yg){
  double x = xg*resolution+offsetX;
  double y = yg*resolution+offsetY;

  PositionVector coordinates;

  coordinates.PosX = x;
  coordinates.PosY = y;

  return coordinates;
}

double point_dist(double x1, double y1, double x2, double y2)
// gives squared distance between points p1 and p2
{
  // reusing vars -- with so many calls we might as well be cheap
  x1 -= x2;
  y1 -= y2;

  // multiplication instead of power is apparently ever so slightly faster
  x1 *= x1;
  y1 *= y1;
  return (x1+y1);
}


double ray_cast(double x, double y, double angle, double error_value) {
  // find starting grid index
  x = (x - offsetX) / resolution;
  y = (y - offsetY) / resolution;

  double x_start = x;
  double y_start = y;

  int x_discrete = std::floor(x);
  int y_discrete = std::floor(y);

  double v_x = cos(angle);
  double v_y = sin(angle);

  // TODO: boundary check (particles could for example move out of the grid map)
  int step_x = std::abs(angle) <= M_PI/2 ? 1:-1;
  int step_y = angle > 0 ? 1:-1;

  // given the ray (x,y)+t*(vx,vy), check for which t the next cell in x and y direction is reached
  double x_boundary = step_x > 0 ? std::ceil(x) : std::floor(x);
  double y_boundary = step_y > 0 ? std::ceil(y) : std::floor(y);
  double t_max_x = (x_boundary - x) / v_x;
  double t_max_y = (y_boundary - y) / v_y;

  // how far to increase t in x and y to go through a whole grid cell
  double t_delta_x = std::abs(1/v_x);
  double t_delta_y = std::abs(1/v_y);

  // iterate over the line until there is an obstacle (note there should always be one, since we are inside the maze)
  try {
    while(map.at(pos2index(x_discrete,y_discrete)) < 50) {
      if(t_max_x < t_max_y) {
        t_max_x = t_max_x + t_delta_x;
        x_discrete = x_discrete + step_x;
      }
      else {
        t_max_y = t_max_y + t_delta_y;
        y_discrete = y_discrete + step_y;
      }
    }
  }
  catch (...) {
    return error_value;
  }
  return std::sqrt( std::pow(x_discrete+0.5-x_start, 2) + std::pow(y_discrete+0.5-y_start, 2) ) * resolution;

}

void grid_callback(const nav_msgs::OccupancyGrid::ConstPtr occupancy_grid){
  ROS_INFO("we got the map");
  width = occupancy_grid -> info.width;
  height = occupancy_grid -> info.height;
  resolution = occupancy_grid -> info.resolution;
  offsetX = occupancy_grid -> info.origin.position.x;
  offsetY = occupancy_grid -> info.origin.position.y;
  map = occupancy_grid -> data;
  if(fog_map.size() < map.size()){
  fog_map.resize(map.size());
}

}

void get_grid_position(geometry_msgs::TransformStamped transformStamped){

  // position coordinates
  pos_x = transformStamped.transform.translation.x;
  pos_y = transformStamped.transform.translation.y;

/*  pos_x = 2.0;
  pos_y = 0.2;*/

  x_grid = (int)((pos_x-offsetX) / resolution);
  y_grid = (int)((pos_y-offsetY) / resolution);
}



bool explore_callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response){
  ros::NodeHandle n;
  ros::Subscriber grid_subscriber = n.subscribe("/slam/occupancy_grid", 1, grid_callback);
  ros::Publisher fog_map_publisher = n.advertise<nav_msgs::OccupancyGrid>("fog_map", 1);
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  while(DONE == false){
    //TRANSFORM
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("map","base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }

    ros::spinOnce();

    if(width != 0){
    //get current position
    ROS_INFO("get position");
    get_grid_position(transformStamped);

    //mark seen area in fog_map
    ROS_INFO("mark area in fog map");
    int grid_radius = (int)mapped_radius/resolution;
    ROS_INFO("grid radius %d", grid_radius);

    for(int xi = 0; xi <= 25; ++xi){
      for(int yi = 0; yi <= 25; ++yi){
        std::vector<int> xg_values= {x_grid+xi, x_grid+xi, x_grid-xi, x_grid-xi};
        std::vector<int> yg_values= {y_grid+yi, y_grid-yi, y_grid-yi, y_grid+yi};
        for(int i = 0; i < 4; i++){
        int xg = xg_values.at(i);
        int yg = yg_values.at(i);
        if(xg >= 0 && yg >= 0 && xg < width && yg <height){
          PositionVector pos = grid_pos2coordinates(xg, yg);
          double x = pos.PosX;
          double y = pos.PosY;
          int dist = pow(x_grid-xg,2)+pow(y_grid-yg,2);
          bool collision = false;

          double angle2point = atan2(y-pos_y, x-pos_x);
          double ray = ray_cast(pos_x, pos_y, angle2point, 1000);
          if(ray+resolution < sqrt(pow(pos_x-x,2)+pow(pos_y-y,2))){
            collision = true;
          }

          int index = pos2index(xg,yg);
          if(dist < pow(25,2) && collision == false){
            fog_map.at(index) = 30;
          }
          if(map.at(index) > 50 && dist < pow(25,2)){
            fog_map.at(index) = 80;
          }
        }
        }
      }
    }
    fog_map.at(pos2index(x_grid, y_grid)) = 100;

      //find go to cell
      ROS_INFO("find go to cell");
      double smallest_dist = 100000000;
      int next_cell;
      bool found_cell = false;
      for(int x = 0; x < width; x++){
        for(int y = 0; y < height; y++){
          double dist = pow(x-x_grid,2)+pow(y-y_grid,2);
          int index = pos2index(x,y);
          bool collision = false;
          PositionVector point = grid_pos2coordinates(x,y);
          double angle2point = atan2(point.PosY-pos_y, point.PosX-pos_x);
          double ray = ray_cast(pos_x, pos_y, angle2point, 1000);
          if(ray+resolution < sqrt(pow(pos_x-point.PosX,2)+pow(pos_y-point.PosY,2))){
            collision = true;
          }
          if(dist < smallest_dist && fog_map.at(index) < 10 && collision == false){
            smallest_dist = dist;
            next_cell = index;
            found_cell = true;
          }
        }
      }
      PositionVector_int next_grid_pos = index2pos(next_cell);
      PositionVector next_pos = grid_pos2coordinates(next_grid_pos.PosX, next_grid_pos.PosY);

      goto_point(next_pos.PosX, next_pos.PosY);
      fog_map.at(next_cell) = 8;

      if(found_cell == false){
        ROS_INFO("exploration done");
        DONE = true;
      }
      else{
        ROS_INFO("go to cell: %d", next_cell);
        PositionVector_int next_grid_pos = index2pos(next_cell);
        PositionVector next_goal = grid_pos2coordinates(next_grid_pos.PosX, next_grid_pos.PosY);
      }

    //publish the map
    nav_msgs::OccupancyGrid fog_map_msg;
    fog_map_msg.data = fog_map;
    fog_map_msg.info.resolution = resolution;
    fog_map_msg.info.width = width;
    fog_map_msg.info.height = height;
    fog_map_msg.info.origin.position.x = offsetX;
    fog_map_msg.info.origin.position.y = offsetY;
    ROS_INFO("publish fog_map");
    fog_map_publisher.publish(fog_map_msg);
  }
    }

  }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_explore_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  explore_srv = n.advertiseService("/explore", &explore_callback);

  mapped_radius = 0.5;

  DONE = false;
  while(ros::ok)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

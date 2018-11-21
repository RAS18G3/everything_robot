#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

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

ros::NodeHandle* npointer;
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
std::vector<int8_t> map;
std::vector<int8_t> fog_map;

void goto_point(double x, double y){
 //Do something good
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
  double x = xg*resolution;
  double y = yg*resolution;

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

bool point_coll(double x1, double y1, double x2, double y2, int collThresh)
// checks for collision between points by stepping along a vector
{
  int x,y; // for indeces
  double vec_x = x2-x1; // dx
  double vec_y = y2-y1; // dy

  // normalize vector
  double vec_magnitude = 2*sqrt(vec_x*vec_x + vec_y*vec_y);
  vec_x /= vec_magnitude;
  vec_y /= vec_magnitude;

  while(1)
  {

    // step once along vector
    x1 += vec_x;
    y1 += vec_y;
    double d = point_dist(x1,y1,x2,y2);
    //if (point_dist(x1, y1, x2, y2) < 1)

    if(d<0.1)
    // short distance . we are at the goal
    {

      break;
    }
    // basic rounding of indeces (recasting.floor)
    int x_floor = (int) x1;
    int y_floor = (int) y1;

    if (map[x_floor + y_floor*width] > collThresh)
    {
      return true;
    }
  }
  return false; // fall through case
}

bool check_collision(double x1, double y1, double x2, double y2, int collThresh){
  double dx = x2-x1;
  double dy = y2-y1;

  double vec_magnitude = sqrt(pow(dx,2) + pow(dy,2));
  dx = (dx/vec_magnitude)*0.01;;
  dy = (dy/vec_magnitude)*0.01;
  double x = x1;
  double y = y1;
  int x_cell;
  int y_cell;
  double distance = sqrt(pow(x-x2,2)+pow(y-y2,2));
  while(distance < 0.02){
  x = x+dx;
  y = y+dy;
  double distance = sqrt(pow(x-x2,2)+pow(y-y2,2));
  x_cell = (int)((x-offsetX) / resolution);
  y_cell = (int)((y-offsetY) / resolution);
  int idx = pos2index(x_cell, y_cell);
  ROS_INFO("map value: %d", map.at(idx));
  if(map.at(idx) > collThresh){
    return true;
  }
}
 return false;
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

void get_grid_position(){
  /*tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  //TRANSFORM
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","base_link", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
  }

  // position coordinates
  pos_x = transformStamped.transform.translation.x;
  pos_y = transformStamped.transform.translation.y;*/

  pos_x = 1;
  pos_y = 1;

  x_grid = (int)((pos_x-offsetX) / resolution);
  y_grid = (int)((pos_y-offsetY) / resolution);
}



bool explore_callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response){
  ros::NodeHandle n;
  ros::Subscriber grid_subscriber = n.subscribe("/slam/occupancy_grid", 1, grid_callback);
  ros::Publisher fog_map_publisher = n.advertise<nav_msgs::OccupancyGrid>("fog_map", 1);
  ros::Rate loop_rate(10);

  while(DONE == false){
    ros::spinOnce();
    loop_rate.sleep();

    if(width != 0){
    //get current position
    ROS_INFO("get position");
    get_grid_position();

    //mark seen area in fog_map
    ROS_INFO("mark area in fog map");
    int grid_radius = (int)mapped_radius/resolution;

    for(int xi = 0; xi < 25; xi++){
      for(int yi = 0; yi < 25; yi++){
        std::vector<int> xg_values= {x_grid+xi, x_grid+xi, x_grid-xi, x_grid-xi};
        std::vector<int> yg_values= {x_grid+yi, x_grid-yi, x_grid-yi, x_grid+yi};
        for(int i = 0; i < 4; i++){
        int xg = xg_values.at(i);
        int yg = yg_values.at(i);
        if(xg >= 0 && yg >= 0 && xg < width && yg <height){
          PositionVector pos = grid_pos2coordinates(xg, yg);
          double x = pos.PosX;
          double y = pos.PosY;
          int dist = pow(x_grid-xg,2)+pow(y_grid-yg,2);
          bool collision = point_coll(pos_x, pos_y, x, y, 40);
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
          if(dist < smallest_dist && fog_map.at(index) < 10){
            smallest_dist = dist;
            next_cell = index;
            found_cell = true;
          }
        }
      }

      if(found_cell == false){
        ROS_INFO("exploration done");
        DONE = true;
      }
      else{
        ROS_INFO("go to cell: %d", next_cell);
        PositionVector_int next_grid_pos = index2pos(next_cell);
        PositionVector next_goal = grid_pos2coordinates(next_grid_pos.PosX, next_grid_pos.PosY);
      }

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

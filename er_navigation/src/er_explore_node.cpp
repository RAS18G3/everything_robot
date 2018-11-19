#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

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

bool point_coll(double x1, double y1, double x2, double y2, std::vector<int8_t> map, int width, int height, int collThresh)
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
    if(d<1)
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

void grid_callback(const nav_msgs::OccupancyGrid::ConstPtr occupancy_grid){
  int width = occupancy_grid -> info.width;
  int height = occupancy_grid -> info.height;
  double resolution = occupancy_grid -> info.resolution;
  double offsetX = occupancy_grid -> info.origin.position.x;
  double offsetY = occupancy_grid -> info.origin.position.y;
  std::vector<int8_t> map = occupancy_grid -> data;
}

void get_grid_position(){
  tf2_ros::Buffer tfBuffer;
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
  pos_y = transformStamped.transform.translation.y;

  x_grid = (int)((pos_x-offsetX) / resolution);
  y_grid = (int)((pos_y-offsetY) / resolution);
}

int pos2index(int x, int y){
  int grid_pos = y*width+x;
  return grid_pos;
}

bool explore_callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response){
  ros::NodeHandle n;
  ros::Subscriber grid_subscriber = n.subscribe("/slam/occupancy_grid", 1, grid_callback);
  ros::Rate loop_rate(10);

  ros::spinOnce();
  loop_rate.sleep();

  //map of what we have already seen
  int size = width*height;
  std::vector<int> fog_map(size);

  while(DONE == false){
    ros::spinOnce();
    loop_rate.sleep();

    //get current position
    get_grid_position();

    //mark seen area in fog_map
    for(double x = x_grid-mapped_radius; x < x_grid+mapped_radius; x+resolution){
      for(double y = y_grid-mapped_radius; y < y_grid+mapped_radius; y+resolution){
        double dist = point_dist(x, y, pos_x, pos_y); //är nu positioner (ej grid positioner)
        bool collision = point_coll(x, y, pos_x, pos_y, map, width, height, 50); // samma som ovan
        int xg = (int)((x-offsetX) / resolution);
        int yg = (int)((y-offsetY) / resolution);
        int index = pos2index(xg,yg);
        if(dist < pow(mapped_radius,2) && collision == false){
          fog_map.at(index) = 30;
          }
          if(map.at(index) > 50){
            fog_map.at(index) = 100;
          }
        }
      }
      double smallest_dist = 100000000;
      int next_cell;
      bool found_cell = false;
      for(int x = 0; x < width; x++){
        for(int y = 0; y < height; y++){
          double dist = pow(x-x_grid,2)+pow(y-y_grid,2);
          int index = pos2index(x,y);
          if(dist < smallest_dist && fog_map.at(index) > 10){
            smallest_dist = dist;
            next_cell = index;
            found_cell = true;
          }
        }
      }

      if(found_cell == false){
        DONE = true;
        ROS_INFO("go to cell: %d", next_cell);
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

#ifndef OCCUPANCY_GRID_UTILS_H
#define OCCUPANCY_GRID_UTILS_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

#include <cmath>

// TODO: add namespace

void draw_line(nav_msgs::OccupancyGrid &occupancy_grid, double x_start, double x_end, double y_start, double y_end, int8_t value=100);
nav_msgs::OccupancyGrid merge_maps(nav_msgs::OccupancyGrid &occupancy_grid_1, nav_msgs::OccupancyGrid &occupancy_grid_2);
double ray_cast_update_mult(nav_msgs::OccupancyGrid &occupancy_grid, double x, double y, double angle, double range, double decrease=0.9, double increase=1.1);
double ray_cast_update(nav_msgs::OccupancyGrid &occupancy_grid, double x, double y, double angle, double range, int decrease=3, int increase=10);
double ray_cast(nav_msgs::OccupancyGrid &occupancy_grid, double x, double y, double angle, double error_value=10);
int8_t&  at(nav_msgs::OccupancyGrid &occupancy_grid, int x, int y);
int sign(int x);

#endif

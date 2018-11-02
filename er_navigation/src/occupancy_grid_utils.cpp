#include "occupancy_grid_utils.h"

void draw_line(nav_msgs::OccupancyGrid &occupancy_grid, double x_start, double x_end, double y_start, double y_end, int8_t value) {
  // I made this algorithm up myself, probably doing something like this would be much more efficient
  // but requires much more code: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
  x_start /= occupancy_grid.info.resolution;
  x_end /= occupancy_grid.info.resolution;
  y_start /= occupancy_grid.info.resolution;
  y_end /= occupancy_grid.info.resolution;
  double delta_x = x_end-x_start;
  double delta_y = y_end-y_start;

  int steps = round(sqrt(pow(delta_x, 2) + pow(delta_y,2))*2+1);
  double x = x_start;
  double y = y_start;

  for(int i=0; i<=steps; ++i) {
    x = x_start + delta_x * i / (double)steps;
    y = y_start + delta_y * i / (double)steps;
    int8_t& data = at(occupancy_grid, round(x), round(y));
    data = value;
  }
}

int8_t&  at(nav_msgs::OccupancyGrid &occupancy_grid, int x, int y)
{
  occupancy_grid.data[y*occupancy_grid.info.width + x];
}

int sign(int x) {
  return x>=0 ? 1 : -1;
}

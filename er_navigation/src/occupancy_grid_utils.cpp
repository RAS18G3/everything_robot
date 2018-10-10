#include "occupancy_grid_utils.h"

void bresenham_line(nav_msgs::OccupancyGrid &occupancy_grid, double x_start, double x_end, double y_start, double y_end) {

}

int8_t&  at(nav_msgs::OccupancyGrid &occupancy_grid, int x, int y) {
  occupancy_grid.data[y*occupancy_grid.info.width + x];
}

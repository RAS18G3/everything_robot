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

double ray_cast_update_mult(nav_msgs::OccupancyGrid &occupancy_grid, double x, double y, double angle, double range, double decrease, double increase) {
  // find starting grid index
  x = (x - occupancy_grid.info.origin.position.x) / occupancy_grid.info.resolution;
  y = (y - occupancy_grid.info.origin.position.y) / occupancy_grid.info.resolution;

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
    while(std::sqrt(std::pow(x_discrete - x,2) + std::pow(y_discrete - y,2)) * occupancy_grid.info.resolution < range - occupancy_grid.info.resolution) {
      if(at(occupancy_grid, x_discrete, y_discrete) < 100 )
        at(occupancy_grid, x_discrete, y_discrete) *= decrease;
      if(at(occupancy_grid, x_discrete, y_discrete) < 5)
        at(occupancy_grid, x_discrete, y_discrete) = 5;


      // go to next cell
      if(t_max_x < t_max_y) {
        t_max_x = t_max_x + t_delta_x;
        x_discrete = x_discrete + step_x;
      }
      else {
        t_max_y = t_max_y + t_delta_y;
        y_discrete = y_discrete + step_y;
      }
    }
    at(occupancy_grid, x_discrete, y_discrete) *= increase;
    if(at(occupancy_grid, x_discrete, y_discrete) > 100)
      at(occupancy_grid, x_discrete, y_discrete) = 100;
  }

  catch (...) {
    return 100;
  }
  return std::sqrt( std::pow(x_discrete+0.5-x_start, 2) + std::pow(y_discrete+0.5-y_start, 2) ) * occupancy_grid.info.resolution;
}

double ray_cast_update(nav_msgs::OccupancyGrid &occupancy_grid, double x, double y, double angle, double range, int decrease, int increase) {
  // find starting grid index
  x = (x - occupancy_grid.info.origin.position.x) / occupancy_grid.info.resolution;
  y = (y - occupancy_grid.info.origin.position.y) / occupancy_grid.info.resolution;

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
    while(std::sqrt(std::pow(x_discrete - x,2) + std::pow(y_discrete - y,2)) * occupancy_grid.info.resolution < range - occupancy_grid.info.resolution) {
      if(at(occupancy_grid, x_discrete, y_discrete) == 100)
        break;
      at(occupancy_grid, x_discrete, y_discrete) -= decrease;
      if(at(occupancy_grid, x_discrete, y_discrete) < 0)
        at(occupancy_grid, x_discrete, y_discrete) = 0;


      // go to next cell
      if(t_max_x < t_max_y) {
        t_max_x = t_max_x + t_delta_x;
        x_discrete = x_discrete + step_x;
      }
      else {
        t_max_y = t_max_y + t_delta_y;
        y_discrete = y_discrete + step_y;
      }
    }
    if(at(occupancy_grid, x_discrete, y_discrete) != 100) {
      at(occupancy_grid, x_discrete, y_discrete) += increase;
      if(at(occupancy_grid, x_discrete, y_discrete) > 99)
        at(occupancy_grid, x_discrete, y_discrete) = 99;
    }
  }

  catch (...) {
    return 100;
  }
  return std::sqrt( std::pow(x_discrete+0.5-x_start, 2) + std::pow(y_discrete+0.5-y_start, 2) ) * occupancy_grid.info.resolution;
}

/* this will approximate the distance between x and y and the first obstacle in direction of angle
 * the paper describing the algorithm can be found in docs/amanatides.pdf
 *
 * angle is assumed to be in [-pi;pi]
 */
double ray_cast(nav_msgs::OccupancyGrid &occupancy_grid, double x, double y, double angle, double error_value) {
  // find starting grid index
  x = (x - occupancy_grid.info.origin.position.x) / occupancy_grid.info.resolution;
  y = (y - occupancy_grid.info.origin.position.y) / occupancy_grid.info.resolution;

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
    while(at(occupancy_grid, x_discrete, y_discrete) < 50) {
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
  return std::sqrt( std::pow(x_discrete+0.5-x_start, 2) + std::pow(y_discrete+0.5-y_start, 2) ) * occupancy_grid.info.resolution;

}

int8_t&  at(nav_msgs::OccupancyGrid &occupancy_grid, int x, int y) {
  if(x >= occupancy_grid.info.width || x < 0)
    throw std::out_of_range ("x out of map boundaries");
  else if (y >= occupancy_grid.info.height || y < 0)
    throw std::out_of_range ("y out of map boundaries");
  else
    occupancy_grid.data[y*occupancy_grid.info.width + x];

}

int sign(int x) {
  return x>=0 ? 1 : -1;
}

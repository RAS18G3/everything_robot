#include "map_reader.h"

MapReader::MapReader() {

}

MapReader::MapReader(std::string path_to_map) {
  ROS_INFO_STREAM("Trying to read map at " << path_to_map);

  std::ifstream ifs;
  double xstart, xend, ystart, yend;
  std::string line;

  try {
    ifs.open(path_to_map);
    while(!ifs.eof()) {
      if (ifs.fail()) {
        throw 1;
      }
      std::getline(ifs, line);

      // skip comments
      if(line[0] == '#' or ifs.eof())
        continue;

      // read the coordinates of the wall in the predefined format
      std::istringstream iss(line);
      iss >> xstart >> ystart >> xend >> yend;

      // store all these coordinates inside the walls vector
      walls.emplace_back(xstart, xend, ystart, yend);

      ROS_DEBUG_STREAM(xstart << " " << ystart << " " << xend << " " << yend);
    }
    ROS_INFO_STREAM("Map succesfully read.");
  }
  catch (...) {
    ROS_ERROR("Unknown error when reading the map, check the file path");
  }
}

MapReader::~MapReader() {

}

nav_msgs::OccupancyGrid MapReader::occupancy_grid(double margin) {
  nav_msgs::OccupancyGrid occupancy_grid_msg = nav_msgs::OccupancyGrid();

  double max_x = 0;
  double max_y = 0;
  double min_x = 0;
  double min_y = 0;
  for(auto it = walls.begin(); it != walls.end(); ++it) {
    if(it->x_start > max_x) {
      max_x = it->x_start;
    }
    if(it->x_start < min_x) {
      min_x = it->x_start;
    }
    if(it->x_end > max_x) {
      max_x = it->x_end;
    }
    if(it->x_end < min_x) {
      min_x = it->x_end;
    }
    if(it->y_start > max_y) {
      max_y = it->y_start;
    }
    if(it->y_start < min_y) {
      min_y = it->y_start;
    }
    if(it->y_end > max_y) {
        max_y = it->y_end;
    }
    if(it->y_end < min_y) {
        min_y = it->y_end;
    }
  }

  if (min_y < 0) {
    for(auto it = walls.begin(); it != walls.end(); ++it) {
      it->y_start -= min_y;
      it->y_end -= min_y;
      max_y -= min_y;
    }
  }

  if (min_x < 0) {
    for(auto it = walls.begin(); it != walls.end(); ++it) {
      it->x_start -= min_x;
      it->x_end -= min_x;
      max_x -= min_x;
    }
  }

  double width = max_x;
  double height = max_y;
  double resolution = 0.02;

  int8_t unknown_value = 5;

  occupancy_grid_msg.header.frame_id = "/map";

  occupancy_grid_msg.info.resolution = resolution;
  occupancy_grid_msg.info.width = (width+2*margin)/resolution;
  occupancy_grid_msg.info.height = (height+2*margin)/resolution;

  occupancy_grid_msg.info.origin.position.x = -margin;
  occupancy_grid_msg.info.origin.position.y = -margin;

  occupancy_grid_msg.data = std::vector<int8_t>(occupancy_grid_msg.info.width * occupancy_grid_msg.info.height, unknown_value);

  for(auto it = walls.begin(); it != walls.end(); ++it) {
    draw_line(occupancy_grid_msg, it->x_start+margin, it->x_end+margin, it->y_start+margin, it->y_end+margin);
  }

  return occupancy_grid_msg;
}

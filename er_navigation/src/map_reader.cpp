#include "map_reader.h"

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

nav_msgs::OccupancyGrid MapReader::occupancy_grid() const {
  nav_msgs::OccupancyGrid occupancy_grid_msg = nav_msgs::OccupancyGrid();

  for(auto it = walls.begin(); it != walls.end(); ++it) {
    bresenham_line(occupancy_grid_msg, it->x_start, it->x_end, it->y_start, it->y_end);
  }

  return occupancy_grid_msg;
}

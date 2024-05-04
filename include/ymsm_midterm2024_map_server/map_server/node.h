#ifndef YMSM_MIDTERM2024_MAP_SERVER_MAP_SERVER_NODE_H_
#define YMSM_MIDTERM2024_MAP_SERVER_MAP_SERVER_NODE_H_

#include "ros/ros.h"

namespace ymsm_midterm2024_map_server::map_server
{

class Node : public ros::NodeHandle
{
public:
  Node();

private:
  struct Point {
    double x, y;
  } point_min_, point_max_;

  double wall_thickness_;

  double dissolution_;

  void publish_map();

  ros::Publisher map_publisher_;
};

}

#endif
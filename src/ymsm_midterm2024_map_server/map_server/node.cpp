#include "ymsm_midterm2024_map_server/map_server/node.h"

#include <algorithm>

#include "nav_msgs/OccupancyGrid.h"

namespace ymsm_midterm2024_map_server::map_server
{

Node::Node() :
  ros::NodeHandle(),
  point_min_({0.0, -2.0}),
  point_max_({5.0, +2.0}),
  wall_thickness_(0.0),
  dissolution_(10.0),
  map_publisher_(this->advertise<nav_msgs::OccupancyGrid>("map", 1, true))
{
  this->publish_map();
}


void Node::publish_map()
{
  nav_msgs::OccupancyGrid map_msg;
  map_msg.header.frame_id = "map";
  map_msg.header.seq = 0;
  map_msg.header.stamp = ros::Time::now();

  map_msg.info.resolution = 1.0 / dissolution_;
  using MapDataValue = decltype(map_msg.data)::value_type;
  const MapDataValue effective_area_width = (point_max_.x - point_min_.x) * dissolution_;
  const MapDataValue effective_area_height = (point_max_.y - point_min_.y) * dissolution_;
  const MapDataValue wall_size = wall_thickness_ * dissolution_;
  map_msg.info.width = effective_area_width + 2 * wall_size;
  map_msg.info.height = effective_area_height + 2 * wall_size;
  map_msg.info.map_load_time = map_msg.header.stamp;
  map_msg.info.origin.position.x = point_min_.x - wall_thickness_;
  map_msg.info.origin.position.y = point_min_.y - wall_thickness_;
  map_msg.info.origin.orientation.x = 0.0;
  map_msg.info.origin.orientation.y = 0.0;
  map_msg.info.origin.orientation.z = 0.0;
  map_msg.info.origin.orientation.w = 1.0;

  map_msg.data.resize(map_msg.info.width * map_msg.info.height);
  MapDataValue w = 0, h = 0;
  for (auto & data : map_msg.data) {
    if (w < wall_size || wall_size + effective_area_width < w
      || h < wall_size || wall_size + effective_area_height < h)
    {
      data = 100;
    }
    else
    {
      data = 0;
    }

    ++w;
    if (w == map_msg.info.width) {
      ++h;
      w = 0;
    }
  }

  map_publisher_.publish(map_msg);
}

}
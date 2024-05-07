#include "ymsm_midterm2024_map_server/map_server/node.h"

#include <algorithm>

#include "nav_msgs/OccupancyGrid.h"

namespace ymsm_midterm2024_map_server::map_server
{

using namespace std::string_literals;

Node::Node() :
  ros::NodeHandle(),
  pnh_("~"),
  seq_(0),
  map_publisher_(this->advertise<nav_msgs::OccupancyGrid>("map", 1, true))
{
  this->publish_map();
}


void Node::publish_map()
{
  nav_msgs::OccupancyGrid map_msg;
  map_msg.header.frame_id = pnh_.param("frame_id", "map"s);
  map_msg.header.stamp = ros::Time::now();
  map_msg.header.seq = seq_;

  const auto wall_thickness = pnh_.param("wall_thickness", 0.0);
  const auto dissolution = pnh_.param("dissolution", 10.0);
  const auto x_min = pnh_.param("x_min", +0.0);
  const auto x_max = pnh_.param("x_max", +5.0);
  const auto y_min = pnh_.param("y_min", -2.0);
  const auto y_max = pnh_.param("y_max", +2.0);
  const std::size_t effective_area_width = (x_max - x_min) * dissolution;
  const std::size_t effective_area_height = (y_max - y_min) * dissolution;
  const std::size_t wall_size = wall_thickness * dissolution;
  map_msg.info.resolution = 1.0 / dissolution;
  map_msg.info.width = effective_area_width + 2 * wall_size;
  map_msg.info.height = effective_area_height + 2 * wall_size;
  map_msg.info.map_load_time = map_msg.header.stamp;
  map_msg.info.origin.position.x = x_min - wall_thickness;
  map_msg.info.origin.position.y = y_min - wall_thickness;
  map_msg.info.origin.position.z = 0.0;
  map_msg.info.origin.orientation.x = 0.0;
  map_msg.info.origin.orientation.y = 0.0;
  map_msg.info.origin.orientation.z = 0.0;
  map_msg.info.origin.orientation.w = 1.0;

  map_msg.data.resize(map_msg.info.width * map_msg.info.height);
  std::size_t w = 0, h = 0;
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

  ++seq_;
}

}
#ifndef YMSM_MIDTERM2024_MAP_SERVER_MAP_SERVER_NODE_H_
#define YMSM_MIDTERM2024_MAP_SERVER_MAP_SERVER_NODE_H_

#include <cstdint>

#include "ros//node_handle.h"

namespace ymsm_midterm2024_map_server::map_server
{

class Node : public ros::NodeHandle
{
public:
  Node();

private:
  ros::NodeHandle pnh_;
  std::uint32_t seq_;

  void publish_map();

  ros::Publisher map_publisher_;
};

}

#endif
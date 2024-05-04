#include "ymsm_midterm2024_map_server/map_server/node.h"

int main(int argc, char **argv)
{
  //ノードの初期化
  ros::init(argc, argv, "map_server");
  ymsm_midterm2024_map_server::map_server::Node node;
  ros::spin();
  return 0;
}
#include "thrust2throttle_controller.hpp"
#include "ros/ros.h"
#include "ros_utils_lib/ros_utils.hpp"

int main(int argc, char **argv)
{

  ros::init(argc, argv, ros_utils_lib::getNodeName("thrust_controller_node"));
  std::cout << "Node starting "<< std::endl;
  Thrust2throttleController thrust_controller;
  thrust_controller.setup();
  ros::Rate r(200);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  
  return 0;
}

#include <ros/ros.h>
#include "turtlebot_demo/turtlebotController.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "turtlebot_controller");
  ros::NodeHandle nodeHandle("~");

  turtlebot_controller::TurtlebotController turtlebotController(nodeHandle);

  ros::spin();
  return 0;
}

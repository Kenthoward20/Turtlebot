#include <ros/ros.h>
#include "turtlebot3_controller/turtlebot3Controller.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "turtlebot_controller");
  ros::NodeHandle nodeHandle("~");

  turtlebot3_controller::Turtlebot3Controller turtlebot3Controller(nodeHandle);

  ros::spin();
  return 0;
}

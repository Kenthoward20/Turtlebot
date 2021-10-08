#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <time.h>

namespace turtlebot3_controller {

/**
 * Class containing the Turtlebot Controller
 */
class Turtlebot3Controller {
 public:
  /** Constructor */
  Turtlebot3Controller(ros::NodeHandle& nodeHandle);

  /** Destructor */
  virtual ~Turtlebot3Controller();

 private:
  bool readParameters();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void publishVelocity(float angle);

  ros::NodeHandle nodeHandle_;
  ros::Subscriber scanSubscriber_;
  ros::Publisher velPublisher_;
  std::string scanTopic_;
  int subscriberQueueSize_;
  float xVel_,kpAng_, kiAng_, kdAng_, prevAngle_, sumErrorAngle_;
  geometry_msgs::Twist velMsg_;
};

}
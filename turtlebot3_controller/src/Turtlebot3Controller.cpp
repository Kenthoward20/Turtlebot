#include <cmath>
#include <algorithm>
#include <turtlebot3_controller/turtlebot3Controller.hpp>

namespace turtlebot3_controller {

Turtlebot3Controller::Turtlebot3Controller(ros::NodeHandle &nodeHandle) :
		nodeHandle_(nodeHandle), subscriberQueueSize_(10), 
		scanTopic_("/scan"), xVel_(0.5), kpAng_(20) , kiAng_(20), kdAng_(20){
	if (!readParameters()) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}
	scanSubscriber_ = nodeHandle_.subscribe(scanTopic_, subscriberQueueSize_,
			&Turtlebot3Controller::scanCallback, this);

    velPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 0);
}

Turtlebot3Controller::~Turtlebot3Controller() {
}

bool Turtlebot3Controller::readParameters() {
	bool success = true;
	success &= nodeHandle_.getParam(
			"/turtlebot_controller/scan_subscriber_topic_name", scanTopic_);
	success &= nodeHandle_.getParam(
			"/turtlebot_controller/scan_subscriber_queue_size",
			subscriberQueueSize_);
    success &= nodeHandle_.getParam(
			"/turtlebot_controller/x_vel", xVel_);
	success &= nodeHandle_.getParam(
			"/turtlebot_controller/kp", kpAng_);
	success &= nodeHandle_.getParam(
			"/turtlebot_controller/ki", kiAng_);
	success &= nodeHandle_.getParam(
			"/turtlebot_controller/kd", kdAng_);
	return success;
}


void Turtlebot3Controller::scanCallback(
		const sensor_msgs::LaserScan::ConstPtr &msg) {
	/* The code below uses C++ 11. The equivalent code using C++ 98 which is more non C++ user friendly is commented out
    
    auto minDistanceIterator = std::min_element(msg->ranges.begin(), msg->ranges.end());
    auto minDistanceIndex = std::distance(msg->ranges.begin(), minDistanceIterator);
    const auto distance = *minDistanceIterator;
	auto angle = msg->angle_min + msg->angle_increment * minDistanceIndex;
	*/
    
    }


void Turtlebot3Controller::publishVelocity(){

}

}
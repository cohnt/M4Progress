#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	geometry_msgs::Point pos = msg->pose.pose.position;

	ROS_INFO("x=%f, y=%f, z=%f", pos.x, pos.y, pos.z);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_listener");
	ros::NodeHandle odomNodeHandle;
	ros::Subscriber odomSubscriber = odomNodeHandle.subscribe("odom", 1000, odomCallback);
	ros::spin();

	return 0;
}
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>

bool needOdom = true;
bool needBase = false;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	if(!needOdom) {
		return;
	}
	else {
		needOdom = false;
		needBase = true;
	}
	const geometry_msgs::Point pos = msg->pose.pose.position;
	const float x = pos.x;
	const float y = pos.y;
	const float z = pos.z;

	std::cout << x << "," << y << "," << z << ",";

	//ROS_INFO("x=%f, y=%f, z=%f", pos.x, pos.y, pos.z);
}
void baseCallBack(const sensor_msgs::LaserScan::ConstPtr& msg) {
	if(!needBase) {
		return;
	}
	else {
		needOdom = true;
		needBase = false;
	}
	const float angle_min = msg->angle_min;
	const float angle_max = msg->angle_max;
	const float angle_increment = msg->angle_increment;
	const std::vector<float> ranges = msg->ranges;

	/*ROS_INFO("angle_min=%f, angle_max=%f, angle_increment=%f", angle_min, angle_max, angle_increment);
	for(int i=0; i<ranges.size(); ++i) {
		ROS_INFO("%f", ranges[i]);
	}*/

	std::cout << angle_min << "," << angle_max << "," << angle_increment;
	for(int i=0; i<ranges.size(); ++i) {
		std::cout << "," << ranges[i];
	}
	std::cout << "\n";
}

int main(int argc, char **argv) {
	std::cout << "odom_x,odom_y,odom_z,angle_min,angle_max,angle_increment,ranges\n";

	ros::init(argc, argv, "odom_listener");
	ros::NodeHandle odomNodeHandle;
	ros::Subscriber odomSubscriber = odomNodeHandle.subscribe("odom", 1000, odomCallback);
	ros::NodeHandle baseNodeHandle;
	ros::Subscriber baseSubscriber = baseNodeHandle.subscribe("base_scan", 1000, baseCallBack);
	ros::spin();

	return 0;
}
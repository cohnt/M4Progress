#include "data_message.h"

dataMessage::dataMessage() {
	//
}

dataMessage::dataMessage(geometry_msgs::Pose odom, sensor_msgs::LaserScan base) {
	odometry = odom;
	baseScan = base;
}

void dataMessage::newOdometry(geometry_msgs::Pose odom) {
	odometry = odom;
}

void dataMessage::newBaseScan(sensor_msgs::LaserScan base) {
	baseScan = base;
}

char* dataMessage::stringify() {
	std::string msg;
	msg += std::to_string(odometry.position.x);
	msg += "|";
	msg += std::to_string(odometry.position.y);
	msg += "|";
	msg += std::to_string(odometry.position.z);
	msg += "|";
	msg += std::to_string(odometry.orientation.x);
	msg += "|";
	msg += std::to_string(odometry.orientation.y);
	msg += "|";
	msg += std::to_string(odometry.orientation.z);
	msg += "|";
	msg += std::to_string(odometry.orientation.w);
	msg += "|";
	msg += std::to_string(baseScan.angle_min);
	msg += "|";
	msg += std::to_string(baseScan.angle_max);
	msg += "|";
	msg += std::to_string(baseScan.angle_increment);
	msg += "|";
	msg += std::to_string(baseScan.ranges[0]);
	for(int i=1; i<baseScan.ranges.size(); ++i) {
		msg += ",";
		msg += std::to_string(baseScan.ranges[i]);
	}
	return strdup(msg.c_str());
}
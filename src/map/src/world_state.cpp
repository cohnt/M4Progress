#include <math.h>

#include "world_state.h"

worldState::worldState() {
	odometry = *(new geometry_msgs::Pose());
	baseScan = *(new sensor_msgs::LaserScan());

	worldState::convertToRobotFrame();
}
worldState::worldState(geometry_msgs::Pose odom, sensor_msgs::LaserScan base) {
	odometry = odom;
	baseScan = base;

	worldState::convertToRobotFrame();
	worldState::convertToWorldFrame();
}

void worldState::convertToRobotFrame() {
	const float lidarForwardDistance = 0.23;
	float t, tMin, tInc;
	tMin = baseScan.angle_min;
	tInc = baseScan.angle_increment;
	t = tMin;
	for(int i=0; i<baseScan.ranges.size(); ++i) {
		t += tInc;
		walls[i][0] = baseScan.ranges[i]*cos(t)+lidarForwardDistance; walls[i][1] = -1*baseScan.ranges[i]*sin(t);
	}
	for(int i=baseScan.ranges.size(); i<1000; ++i) {
		walls[i][0] = odometry.position.x;
		walls[i][1] = odometry.position.y;
	}
}
float worldState::convertToWorldFrame() {
	float theta = static_cast<float>(atan2(2*((odometry.orientation.x*odometry.orientation.y) + (odometry.orientation.z*odometry.orientation.w)), 1-(2*((odometry.orientation.y*odometry.orientation.y) + (odometry.orientation.z*odometry.orientation.z)))));
	for(int i=0; i<sizeof(walls)/sizeof(walls[0]); ++i) {
		walls[i][0] = (walls[i][0] * cos(theta)) - (walls[i][1] * sin(theta)) + odometry.position.x;
		walls[i][1] = (walls[i][0] * sin(theta)) + (walls[i][1] * cos(theta)) + odometry.position.y;
	}
	return theta;
}
void worldState::newOdometry(geometry_msgs::Pose odom) {
	odometry = odom;
	worldState::convertToRobotFrame();
}
float worldState::newBaseScan(sensor_msgs::LaserScan base) {
	baseScan = base;
	worldState::convertToRobotFrame();
	float a = worldState::convertToWorldFrame();
	return a;
}
geometry_msgs::Pose worldState::getOdometry() {
	//[3]
	return odometry;
}
sensor_msgs::LaserScan worldState::getBaseScan() {
	//
	return baseScan;
}
void worldState::getWalls(float (&copyIntoThis)[BASE_SCAN_MAX_NUM_POINTS][2]) {
	for(int i=0; i<BASE_SCAN_MAX_NUM_POINTS; ++i) {
		for(int j=0; j<2; ++j) {
			copyIntoThis[i][j] = walls[i][j];
		}
	}
}
char* worldState::stringify() {
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
	msg += "|[";
	msg += std::to_string(walls[0][0]);
	msg += ",";
	msg += std::to_string(walls[0][1]);
	msg += "]";
	for(int i=1; i<sizeof(walls)/sizeof(walls[0]); ++i) {
		msg +=";[";
		msg += std::to_string(walls[i][0]);
		msg += ",";
		msg += std::to_string(walls[i][1]);
		msg += "]";
	}
	return strdup(msg.c_str());
}
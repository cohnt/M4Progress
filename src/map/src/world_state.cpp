#include <math.h>

#include "world_state.h"
#include "json.hpp"

using json = nlohmann::json;

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
	for(int i=baseScan.ranges.size(); i<BASE_SCAN_MAX_NUM_POINTS; ++i) {
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
char* worldState::makeJSONString() {
	const std::vector<float> position = {odometry.position.x, odometry.position.y, odometry.position.z};
	const std::vector<float> orientation  = {odometry.orientation.x, odometry.orientation.y, odometry.orientation.z, odometry.orientation.w};

	json pose = {
		{"position", position},
		{"orientation", orientation},
		{"angleMin", baseScan.angle_min},
		{"angleMax", baseScan.angle_max},
		{"angleIncrement", baseScan.angle_increment},
		{"ranges", baseScan.ranges}
	};
	pose["walls"] = json::array();

	for(int i=0; i<sizeof(walls)/sizeof(walls[0]); ++i) {

		pose["walls"][i] = json::array();
		pose["walls"][i][0] = walls[i][0];
		pose["walls"][i][1] = walls[i][1];
	}

	std::string poseStr = pose.dump();

	json message = {
		json::array()
	};
	message[0] = json::parse(poseStr);

	std::string msgString = message.dump();

	return strdup(msgString.c_str());
}
#include <math.h>

#include "world_state.h"
#include "json.hpp"

using json = nlohmann::json;

worldState::worldState() {
	odometry = *(new geometry_msgs::Pose());
	baseScan = *(new sensor_msgs::LaserScan());
	theta = 0;
	lidarForwardDistance = 0;

	walls.reserve(BASE_SCAN_MAX_NUM_POINTS);

	worldState::convertToRobotFrame();
}
worldState::worldState(geometry_msgs::Pose odom, sensor_msgs::LaserScan base, double lidarDistance) {
	odometry = odom;
	baseScan = base;
	theta = atan2(2*((odometry.orientation.x*odometry.orientation.y) + (odometry.orientation.z*odometry.orientation.w)), 1-(2*((odometry.orientation.y*odometry.orientation.y) + (odometry.orientation.z*odometry.orientation.z))));
	lidarForwardDistance = lidarDistance;

	walls.reserve(BASE_SCAN_MAX_NUM_POINTS);

	worldState::convertToRobotFrame();
	worldState::convertToWorldFrame();
}

void worldState::convertToRobotFrame() {
	double t, tMin, tInc;
	tMin = baseScan.angle_min;
	tInc = baseScan.angle_increment;
	t = tMin;
	for(int i=0; i<baseScan.ranges.size(); ++i) {
		walls.push_back(
			std::array<double, 3>{
				baseScan.ranges[i]*cos(t)+lidarForwardDistance,
				-1*baseScan.ranges[i]*sin(t),
				1
			}
		);
		t += tInc;
	}
}
double worldState::convertToWorldFrame() {
	for(int i=0; i<walls.size(); ++i) {
		double x = walls[i][0];
		double y = walls[i][1];
		walls[i][0] = ((x * cos(theta)) + (y * sin(-theta))) + odometry.position.x;
		walls[i][1] = ((x * sin(theta)) + (y * cos(theta))) + odometry.position.y;
	}
	for(int i=0; i<walls.size(); ++i) {
		if(walls[i][0] != walls[i][0] || walls[i][1] != walls[i][1] || walls[i][2] != walls[i][2]) {
			walls.erase(walls.begin() + i);
			--i;
		}
	}
	return theta;
}
void worldState::newOdometry(geometry_msgs::Pose odom) {
	odometry = odom;
	theta = atan2(2*((odometry.orientation.x*odometry.orientation.y) + (odometry.orientation.z*odometry.orientation.w)), 1-(2*((odometry.orientation.y*odometry.orientation.y) + (odometry.orientation.z*odometry.orientation.z))));
}
double worldState::newBaseScan(sensor_msgs::LaserScan base) {
	baseScan = base;
	walls.resize(0);
	worldState::convertToRobotFrame();
	double a = worldState::convertToWorldFrame();
	return a;
}
void worldState::setLidarForwardDistance(double d) {
	//
	lidarForwardDistance = d;
}
geometry_msgs::Pose worldState::getOdometry() {
	//
	return odometry;
}
sensor_msgs::LaserScan worldState::getBaseScan() {
	//
	return baseScan;
}
std::vector<std::array<double, 3>> worldState::getWalls() {
	std::vector<std::array<double, 3>> toReturn;
	toReturn.reserve(BASE_SCAN_MAX_NUM_POINTS);
	for(int i=0; i<walls.size(); ++i) {
		toReturn.push_back(walls[i]);
	}
	return toReturn;
}
double worldState::getTheta() {
	//
	return theta;
}
char* worldState::makeJSONString() {
	const std::vector<double> position = {odometry.position.x, odometry.position.y, odometry.position.z};
	const std::vector<double> orientation  = {odometry.orientation.x, odometry.orientation.y, odometry.orientation.z, odometry.orientation.w};

	json pose = {
		{"position", position},
		{"orientation", orientation},
		{"theta", theta},
		{"angleMin", baseScan.angle_min},
		{"angleMax", baseScan.angle_max},
		{"angleIncrement", baseScan.angle_increment},
		{"ranges", baseScan.ranges}
	};
	pose["walls"] = json::array();

	for(int i=0; i<walls.size(); ++i) {
		pose["walls"][i] = json::array();
		pose["walls"][i][0] = walls[i][0];
		pose["walls"][i][1] = walls[i][1];
		pose["walls"][i][2] = walls[i][2];
	}

	std::string poseStr = pose.dump();

	json message = {
		{"type", "SENDDATA"},
		{"data", json::array()}
	};
	message["data"][0] = json::parse(poseStr);

	std::string msgString = message.dump();

	return strdup(msgString.c_str());
}
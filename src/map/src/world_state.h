#ifndef WORLD_STATE
#define WORLD_STATE

#define BASE_SCAN_MAX_NUM_POINTS 662

#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/LaserScan.h"

class worldState {
	private:
		geometry_msgs::Pose odometry;
		sensor_msgs::LaserScan baseScan;
		double theta;
		double lidarForwardDistance;
		void convertToRobotFrame();
		double convertToWorldFrame();

	public:
		std::vector<std::array<double, 3>> walls;
		
		worldState();
		worldState(geometry_msgs::Pose odom, sensor_msgs::LaserScan base, double lidarDistance);

		void newOdometry(geometry_msgs::Pose odom, std::array<std::array<double, 3>, 3> slam);
		double newBaseScan(sensor_msgs::LaserScan base);
		void setLidarForwardDistance(double d);
		geometry_msgs::Pose getOdometry();
		sensor_msgs::LaserScan getBaseScan();
		std::vector<std::array<double, 3>> getWalls();
		double getTheta();

		char* makeJSONString();
};

#endif
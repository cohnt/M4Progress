#ifndef WORLD_STATE
#define WORLD_STATE

#ifndef BASE_SCAN_MAX_NUM_POINTS
#define BASE_SCAN_MAX_NUM_POINTS 662
#endif

#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/LaserScan.h"

class worldState {
	private:
		geometry_msgs::Pose odometry;
		sensor_msgs::LaserScan baseScan;
		double walls[BASE_SCAN_MAX_NUM_POINTS][2];
		double theta;
		double lidarForwardDistance;
		void convertToRobotFrame();
		double convertToWorldFrame();

	public:
		worldState();
		worldState(geometry_msgs::Pose odom, sensor_msgs::LaserScan base, double lidarDistance);

		void newOdometry(geometry_msgs::Pose odom);
		double newBaseScan(sensor_msgs::LaserScan base);
		void setLidarForwardDistance(double d);
		geometry_msgs::Pose getOdometry();
		sensor_msgs::LaserScan getBaseScan();
		void getWalls(double (&copyIntoThis)[BASE_SCAN_MAX_NUM_POINTS][2]);
		double getTheta();

		char* makeJSONString();
};

#endif
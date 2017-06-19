#ifndef WORLD_STATE
#define WORLD_STATE

#define BASE_SCAN_MAX_NUM_POINTS 1000

#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/LaserScan.h"

class worldState {
	private:

	public:
		geometry_msgs::Pose odometry;
		sensor_msgs::LaserScan baseScan;
		float walls[BASE_SCAN_MAX_NUM_POINTS][2];
		void convertToRobotFrame();
		
		worldState();
		worldState(geometry_msgs::Pose odom, sensor_msgs::LaserScan base);

		void newOdometry(geometry_msgs::Pose odom);
		void newBaseScan(sensor_msgs::LaserScan base);
		geometry_msgs::Pose getOdometry();
		sensor_msgs::LaserScan getBaseScan();
		void getWalls(float (&copyIntoThis)[BASE_SCAN_MAX_NUM_POINTS][2]);

		char* stringify();
};

#endif
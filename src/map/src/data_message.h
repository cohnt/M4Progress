#ifndef DATA_MESSAGE
#define DATA_MESSAGE

#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/LaserScan.h"

class dataMessage {
	private:
		geometry_msgs::Pose odometry;
		sensor_msgs::LaserScan baseScan;

	public:
		dataMessage();
		dataMessage(geometry_msgs::Pose odom, sensor_msgs::LaserScan base);

		void newOdometry(geometry_msgs::Pose odom);
		void newBaseScan(sensor_msgs::LaserScan base);

		char* stringify();
};

#endif
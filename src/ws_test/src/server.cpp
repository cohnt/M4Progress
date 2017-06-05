#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"

#include <iostream>

#include "Util.h"
#include "WebSocketServer.h"

bool needOdom;

class EchoServer : public WebSocketServer {
	public: 
		EchoServer(int port);
		~EchoServer();
		virtual void onConnect(int socketID);
		virtual void onMessage(int socketID, const string& data);
		virtual void onDisconnect(int socketID);
		virtual void onError(int socketID, const string& message);
};

EchoServer::EchoServer(int port) : WebSocketServer(port) {}
EchoServer::~EchoServer() {}
void EchoServer::onConnect(int socketID) {
	Util::log("New connection");
}
void EchoServer::onMessage(int socketID, const string& data) {
	// Reply back with the same message
	Util::log("Received: " + data);
	this->send(socketID, data);
}
void EchoServer::onDisconnect(int socketID) {
	Util::log("Disconnect");
}
void EchoServer::onError(int socketID, const string& message) {
	Util::log("Error: " + message);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	if(needOdom != false) {
		needOdom = false;

		const geometry_msgs::Point pos = msg->pose.pose.position;
		const float x = pos.x;
		const float y = pos.y;
		const float z = pos.z;

		std::cout << x << "," << y << "," << z << ",\n";
	}
}
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	if(needOdom != true) {
		needOdom = true;

		const float angle_min = msg->angle_min;
		const float angle_max = msg->angle_max;
		const float angle_increment = msg->angle_increment;
		const std::vector<float> ranges = msg->ranges;

		std::cout << "angle_min:" << angle_min << " angle_max:" << angle_max << " angle_increment:" << angle_increment << "\n";
		std::cout << "ranges:" << ranges[0];
		for(int i=0; i<ranges.size(); ++i) {
			std::cout << "," << ranges[i];
		}
		std::cout << "\n";
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "send_data_to_page");
	ros::NodeHandle odomNodeHandle;
	ros::Subscriber odomSubscriber = odomNodeHandle.subscribe("odom", 1000, odomCallback);
	ros::NodeHandle scanNodeHandle;
	ros::Subscriber scanSubscriber = scanNodeHandle.subscribe("base_scan", 1000, scanCallback);

	EchoServer es = EchoServer(8080);
	es.run();

	ros::spin();

	return 0;
}
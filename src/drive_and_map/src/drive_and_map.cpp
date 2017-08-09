#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"

#include <iostream>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <math.h>

#include "json.hpp"

typedef websocketpp::server<websocketpp::config::asio> server;
typedef server::message_ptr message_ptr;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

using json = nlohmann::json;

void on_message(server* s, websocketpp::connection_hdl hdl, message_ptr msg) {
	//
	//
}
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	//
	//
}
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	//
	//
}

void startServer() {
	std::cout << "Starting WebSocket server...\t\t\t";
	server wsServer;
	wsServer.set_access_channels(websocketpp::log::alevel::all);
	wsServer.clear_access_channels(websocketpp::log::alevel::frame_payload);
	wsServer.set_reuse_addr(true);
	wsServer.init_asio();
	wsServer.set_message_handler(bind(&on_message,&wsServer,::_1,::_2));
	wsServer.listen(9002);
	wsServer.start_accept();
	std::cout << "Done!" << std::endl;

	std::thread wsServerThread(&server::run, &wsServer);
	wsServerThread.detach();
	const std::thread::id id = wsServerThread.get_id();
	std::cout << "Server thread id: " << id << std::endl;

	return;
}
void startROS(int &argc, char** &argv) {
	std::cout << "Starting ROS node...\t\t\t";
	ros::init(argc, argv, "drive_and_map");
	ros::NodeHandle odomNodeHandle;
	ros::Subscriber odomSubscriber = odomNodeHandle.subscribe("odom", 1000, odomCallback);
	ros::NodeHandle scanNodeHandle;
	ros::Subscriber scanSubscriber = scanNodeHandle.subscribe("base_scan", 1000, scanCallback);
	std::cout << "Done!" << std::endl;

	ros::spin();

	return;
}

int main(int argc, char **argv) {
	std::cout << "Starting drive_and_map.cpp" << std::endl;

	startServer();
	startROS(argc, argv);

	while(ros::ok()) {
		//
	}

	return 0;
}
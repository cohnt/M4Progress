#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"

#include <iostream>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

typedef websocketpp::server<websocketpp::config::asio> server;

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

typedef server::message_ptr message_ptr;

bool needOdom = true; //We want an odometry message first.

void on_message(server* s, websocketpp::connection_hdl hdl, message_ptr msg) {
	std::cout << "on_message called with hdl: " << hdl.lock().get() << " and message: " << msg->get_payload() << std::endl;

	// check for a special command to instruct the server to stop listening so
	// it can be cleanly exited.
	if(msg->get_payload() == "stop-listening") {
		s->stop_listening();
		return;
	}

	try {
		s->send(hdl, msg->get_payload(), msg->get_opcode());
	}
	catch (const websocketpp::lib::error_code& e) {
		std::cout << "Echo failed because: " << e << "(" << e.message() << ")" << std::endl;
	}
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
	server echoServer;
	echoServer.set_access_channels(websocketpp::log::alevel::all);
	echoServer.clear_access_channels(websocketpp::log::alevel::frame_payload);
	echoServer.init_asio();
	echoServer.set_message_handler(bind(&on_message,&echoServer,::_1,::_2));
	echoServer.listen(9002);
	echoServer.start_accept();

	std::thread wsServer(&server::run, &echoServer);
	wsServer.detach();

	ros::init(argc, argv, "send_data_to_page");
	ros::NodeHandle odomNodeHandle;
	ros::Subscriber odomSubscriber = odomNodeHandle.subscribe("odom", 1000, odomCallback);
	ros::NodeHandle scanNodeHandle;
	ros::Subscriber scanSubscriber = scanNodeHandle.subscribe("base_scan", 1000, scanCallback);
	ros::spin();

	echoServer.stop_listening();

	return 0;
}
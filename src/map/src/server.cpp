#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"

#include <iostream>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <math.h>
#include <mutex>

#include "world_state.h"
#include "json.hpp"

typedef websocketpp::server<websocketpp::config::asio> server;

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

using json = nlohmann::json;

typedef server::message_ptr message_ptr;

bool needOdom = true; //We want an odometry message first.
bool serverStarted = false; //Has the server started yet?
geometry_msgs::Pose lastOdomPose; //The last recorded odometry pose.
sensor_msgs::LaserScan lastBaseScan; //The last recorded base scan.
std::vector<worldState> states;
worldState lastWorldState; //The most recent world state.
bool newDataForClient;

std::mutex mutex;

float minPoseTranslationToSave = pow(0.02, 2);
float minPoseRotationToSave = M_PI / 180;

float distanceSquared(std::vector<float> a, std::vector<float> b) {
	float sum = 0;
	for(int i=0; i<a.size(); ++i) {
		sum += pow(a[i]-b[i], 2);
	}
	return sum;
}
bool doSave(worldState state) {
	if(states.size() == 0) {
		return true;
	}
	geometry_msgs::Pose currentPose = state.getOdometry();
	geometry_msgs::Pose oldPose = states[states.size()-1].getOdometry();
	std::vector<float> currentPoseVector = {currentPose.position.x, currentPose.position.y, currentPose.position.z};
	std::vector<float> oldPoseVector = {oldPose.position.x, oldPose.position.y, oldPose.position.z};
	std::cout << "d^2: " << distanceSquared(currentPoseVector, oldPoseVector) << ", min: " << minPoseTranslationToSave << std::endl;
	std::cout << "dTheta: " << abs(state.getTheta()-states[states.size()-1].getTheta()) << ", min: " << minPoseRotationToSave << std::endl;
	return distanceSquared(currentPoseVector, oldPoseVector) || (abs(state.getTheta()-states[states.size()-1].getTheta()) > minPoseRotationToSave);
}
void on_message(server* s, websocketpp::connection_hdl hdl, message_ptr msg) {
	try {
		mutex.lock();
		std::string incomingMsg = msg->get_payload();
		if(incomingMsg == "ready") {
			if(newDataForClient) {
				newDataForClient = false;
				char *outgoingMessage = states[states.size()-1].makeJSONString();
				s->send(hdl, outgoingMessage, msg->get_opcode());
				free(outgoingMessage);
			}
			else {
				s->send(hdl, "wait", msg->get_opcode());
			}
		}
		mutex.unlock();
	}
	catch (const websocketpp::lib::error_code& e) {
		std::cout << "Echo failed because: " << e << "(" << e.message() << ")" << std::endl;
	}
}
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	if(needOdom != false) {
		mutex.lock();
		needOdom = false;
		lastOdomPose = msg->pose.pose;
		mutex.unlock();
	}
}
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	if(needOdom != true) {
		mutex.lock();
		needOdom = true;
		lastBaseScan = *msg;
		lastWorldState.newBaseScan(lastBaseScan);
		lastWorldState.newOdometry(lastOdomPose);
		std::cout << "\t\t\tNumber of saved states: " << states.size() << std::endl;
		if(doSave(lastWorldState)) {
			states.push_back(lastWorldState);
			newDataForClient = true;
		}
		mutex.unlock();
	}
}

int main(int argc, char **argv) {
	server echoServer;
	echoServer.set_access_channels(websocketpp::log::alevel::all);
	echoServer.clear_access_channels(websocketpp::log::alevel::frame_payload);
	echoServer.set_reuse_addr(true);
	echoServer.init_asio();
	echoServer.set_message_handler(bind(&on_message,&echoServer,::_1,::_2));
	echoServer.listen(9002);
	echoServer.start_accept();

	std::thread wsServer(&server::run, &echoServer);
	wsServer.detach();
	serverStarted = true;

	ros::init(argc, argv, "send_data_to_page");
	ros::NodeHandle odomNodeHandle;
	ros::Subscriber odomSubscriber = odomNodeHandle.subscribe("odom", 1000, odomCallback);
	ros::NodeHandle scanNodeHandle;
	ros::Subscriber scanSubscriber = scanNodeHandle.subscribe("base_scan", 1000, scanCallback);
	ros::spin();

	while(ros::ok()) {
		//
	}

	echoServer.stop_perpetual();
	echoServer.stop_listening();

	return 0;
}
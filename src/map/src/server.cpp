#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"

#include <iostream>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <math.h>
#include <algorithm>
#include <mutex>

#include "world_state.h"
#include "scan_match.h"
#include "json.hpp"

#define NUM_STATES_TO_RESERVE 4096
#define STATES_CACHE_SIZE 64

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
std::vector<worldState> unsentStates;
worldState lastWorldState; //The most recent world state.
bool newDataForClient;
bool justGotConfig = false;
std::array<std::array<double, 3>, 3> slamTransform = {
	std::array<double, 3>{1, 0, 0},
	std::array<double, 3>{0, 1, 0},
	std::array<double, 3>{0, 0, 1}
};

std::mutex mutex;

double minPoseTranslationToSave = pow(0.001, 2);
double minPoseRotationToSave = M_PI / 1800;
double lidarDistance = 0.23;

icpConfig config;

double distanceSquared(std::vector<double> a, std::vector<double> b) {
	double sum = 0;
	for(int i=0; i<a.size(); ++i) {
		sum += pow(a[i]-b[i], 2);
	}
	return sum;
}
bool doSave(worldState state) {
	if(states.size() == 0) {
		return true;
	}
	else if(justGotConfig) {
		states.resize(0);
		unsentStates.resize(0);
		justGotConfig = false;
		return true;
	}
	geometry_msgs::Pose currentPose = state.getOdometry();
	geometry_msgs::Pose oldPose = states[states.size()-1].getOdometry();
	std::vector<double> currentPoseVector = {currentPose.position.x, currentPose.position.y, currentPose.position.z};
	std::vector<double> oldPoseVector = {oldPose.position.x, oldPose.position.y, oldPose.position.z};
	std::cout << "d^2: " << distanceSquared(currentPoseVector, oldPoseVector) << ", min: " << minPoseTranslationToSave << std::endl;
	std::cout << "t1: " << states[states.size()-1].getTheta() << " t2: " << state.getTheta() << std::endl;
	std::cout << "dTheta: " << fabs(state.getTheta()-states[states.size()-1].getTheta()) << ", min: " << minPoseRotationToSave << std::endl;
	return distanceSquared(currentPoseVector, oldPoseVector) > minPoseTranslationToSave || (fabs(state.getTheta()-states[states.size()-1].getTheta()) > minPoseRotationToSave);
}
void on_message(server* s, websocketpp::connection_hdl hdl, message_ptr msg) {
	try {
		mutex.lock();
		std::string incomingMsg = msg->get_payload();
		json message = json::parse(incomingMsg);
		if(message["type"] == "REQUESTDATA") {
			if(newDataForClient) {
				assert(unsentStates.size() > 0);
				char *outgoingMessage = unsentStates[0].makeJSONString();
				std::cout << "\t\t\t\t\t\t\t RECEIVED: " << incomingMsg << std::endl;
				std::cout << "\t\t\t\t\t\t\t SENT: " << "(data message)" << std::endl;
				s->send(hdl, outgoingMessage, msg->get_opcode());
				free(outgoingMessage);
				unsentStates.erase(unsentStates.begin());
				newDataForClient = unsentStates.size() > 0;
			}
			else {
				json outgoingMessage = {
					{"type", "WAIT"}
				};
				//std::cout << "\t\t\t\t\t\t\t SENT: " << outgoingMessage.dump() << std::endl;
				s->send(hdl, outgoingMessage.dump(), msg->get_opcode());
			}
		}
		else if(message["type"] == "CONFIG") {
			std::cout << "\t\t\t\t\t\t\t RECEIVED: " << incomingMsg << std::endl;
			minPoseTranslationToSave = message["minPoseTranslationToSave"];
			minPoseRotationToSave = message["minPoseRotationToSave"];
			lidarDistance = message["lidarForwardDistance"];
			lastWorldState.setLidarForwardDistance(lidarDistance);
			config.minICPComparePoints = message["minICPComparePoints"];
			config.maxICPLoopCount = message["maxICPLoopCount"];
			config.icpAverageDistanceTraveledThresholdSquared = message["icpAverageDistanceTraveledThresholdSquared"];
			config.icpNoMovementCounterThreshold = message["icpNoMovementCounterThreshold"];
			config.goodCorrespondenceThresholdSquared = message["goodCorrespondenceThresholdSquared"];
			config.maximumPointMatchDistance = message["maximumPointMatchDistance"];
			config.percentChanceToMatchPoints = message["percentChanceToMatchPoints"];
			config.scanDensityDistanceSquared = message["scanDensityDistanceSquared"];
			json outgoingMessage = {
				{"type", "RECEIVEDCONFIG"}
			};
			std::cout << "\t\t\t\t\t\t\t SENT: " << outgoingMessage.dump() << std::endl;
			s->send(hdl, outgoingMessage.dump(), msg->get_opcode());
			newDataForClient = false;
			justGotConfig = true;
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
		lastWorldState.newOdometry(lastOdomPose, slamTransform);
		lastWorldState.newBaseScan(lastBaseScan);
		std::cout << "\t\t\tNumber of saved states: " << states.size() << std::endl;
		if(doSave(lastWorldState)) {
			if(states.size() != 0) {
				optimizationOutput output = optimizeScan(lastWorldState, states, config);
				std::cout << "ICP Loop Count: " << output.icpLoopCount << std::endl;
				//std::cout << "ICP Average Distances Traveled: ";
				//for(int i=0; i<output.avgD.size(); ++i) {
				//	std::cout << output.avgD[i];
				//}
				//std::cout << std::endl;
				//std::cout << "ICP Transform History: " << std::endl;
				//for(int i=0; i<output.angleHistory.size(); ++i) {
				//	std::cout << "Angle: " << output.angleHistory[i] << "\t Translation:" << output.translationHistory[i][0] << ", " << output.translationHistory[i][1] << std::endl;
				//}
				//std::cout << "Net angle error: " << output.netAngleError << std::endl;
				//std::cout << "Net position error: " << output.netPositionError[0] << ", " << output.netPositionError[1] << std::endl;
				std::cout << "Total angle error: " << atan2(slamTransform[1][0], slamTransform[0][0]) << std::endl;
				std::cout << "Total position error: " << slamTransform[0][2] << ", " << slamTransform[1][2] << std::endl;

				//std::cout << std::endl << std::endl << std::endl << std::endl << std::endl;

				if(output.success) {
					std::cout << "Success? \t\t\t\t\t  __    __   ____    ____ " << std::endl;
					std::cout << "         \t\t\t\t\t  \\ \\  / /  | ___|  /  __|" << std::endl;
					std::cout << "         \t\t\t\t\t   \\ \\/ /   | |_    | |__ " << std::endl;
					std::cout << "         \t\t\t\t\t    |  |    |  _|   \\__  \\" << std::endl;
					std::cout << "         \t\t\t\t\t    |  |    | |__    __| |"<< std::endl;
					std::cout << "         \t\t\t\t\t    |__|    |____|  |____/" << std::endl;
				}
				else {
					std::cout << "Success? \t\t\t\t\t   __    _    _____ " << std::endl;
					std::cout << "         \t\t\t\t\t  |  \\  | |  /  _  \\" << std::endl;
					std::cout << "         \t\t\t\t\t  |   \\ | |  | / \\ |" << std::endl;
					std::cout << "         \t\t\t\t\t  | |\\ \\| |  | | | |" << std::endl;
					std::cout << "         \t\t\t\t\t  | | \\   |  | \\_/ |" << std::endl;
					std::cout << "         \t\t\t\t\t  |_|  \\__|  \\_____/" << std::endl;
				}

				if(output.success) {
					slamTransform = product(slamTransform, output.currentSLAM);
					states.push_back(lastWorldState);
					unsentStates.push_back(lastWorldState);
					newDataForClient = true;
				}
			}
			else {
				states.push_back(lastWorldState);
				unsentStates.push_back(lastWorldState);
				newDataForClient = true;
			}
		}
		mutex.unlock();
	}
}

int main(int argc, char **argv) {
	states.reserve(NUM_STATES_TO_RESERVE);
	unsentStates.reserve(STATES_CACHE_SIZE);

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
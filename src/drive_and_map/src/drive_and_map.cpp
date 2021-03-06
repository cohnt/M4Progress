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

#include "pose.h"
#include "scan_match.h"
#include "json.hpp"

typedef websocketpp::server<websocketpp::config::asio> server;
typedef server::message_ptr message_ptr;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

using json = nlohmann::json;

bool needOdom = true; //We want an odometry message first.
bool serverStarted = false; //Has the server started yet?
geometry_msgs::Pose lastOdomPose; //The last recorded odometry pose.
sensor_msgs::LaserScan lastBaseScan; //The last recorded base scan.
std::vector<pose> poses;
std::vector<pose> unsentPoses;
pose lastWorldState; //The most recent world state.
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

const int rosNodeQueueSize = 1000; //How many messages are cached by each ROS node.
const int wsServerPort = 9002; //The port the WebSocket server is on.
icpConfig config;

double distanceSquared(std::vector<double> a, std::vector<double> b) {
	double sum = 0;
	for(int i=0; i<a.size(); ++i) {
		sum += pow(a[i]-b[i], 2);
	}
	return sum;
}
bool doSave(pose currentPose) {
	//bool doSave(pose currentPose) determines whether or not a pose should be processed and saved.

	if(poses.size() == 0) { //We always save the first pose received.
		return true;
	}
	else if(justGotConfig) {
		poses.resize(0);
		unsentPoses.resize(0);
		justGotConfig = false;
		return true;
	}
	geometry_msgs::Pose currentOdometry = currentPose.getOdometry();
	geometry_msgs::Pose oldOdometry = poses[poses.size()-1].getOdometry();
	std::vector<double> currentPosition = {currentOdometry.position.x, currentOdometry.position.y, currentOdometry.position.z};
	std::vector<double> oldPosition = {oldOdometry.position.x, oldOdometry.position.y, oldOdometry.position.z};
	std::cout << "d^2: " << distanceSquared(currentPosition, oldPosition) << ", min: " << minPoseTranslationToSave << std::endl;
	std::cout << "t1: " << poses[poses.size()-1].getTheta() << " t2: " << currentPose.getTheta() << std::endl;
	std::cout << "dTheta: " << fabs(currentPose.getTheta()-poses[poses.size()-1].getTheta()) << ", min: " << minPoseRotationToSave << std::endl;
	return distanceSquared(currentPosition, oldPosition) > minPoseTranslationToSave || (fabs(currentPose.getTheta()-poses[poses.size()-1].getTheta()) > minPoseRotationToSave);
}

void on_message(server* s, websocketpp::connection_hdl hdl, message_ptr msg) {
	try {
		mutex.lock();
		std::string incomingMsg = msg->get_payload();
		json message = json::parse(incomingMsg);
		if(message["type"] == "REQUESTDATA") {
			if(newDataForClient) {
				assert(unsentPoses.size() > 0);
				char *outgoingMessage = unsentPoses[0].makeJSONString();
				std::cout << "\t\t\t\t\t\t\t RECEIVED: " << incomingMsg << std::endl;
				std::cout << "\t\t\t\t\t\t\t SENT: " << "(data message)" << std::endl;
				s->send(hdl, outgoingMessage, msg->get_opcode());
				free(outgoingMessage);
				unsentPoses.erase(unsentPoses.begin());
				newDataForClient = unsentPoses.size() > 0;
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
		std::cout << "\t\t\tNumber of saved poses: " << poses.size() << std::endl;
		if(doSave(lastWorldState)) {
			if(poses.size() != 0) {
				optimizationOutput output = optimizeScan(lastWorldState, poses, config);
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
					poses.push_back(lastWorldState);
					unsentPoses.push_back(lastWorldState);
					newDataForClient = true;
				}
			}
			else {
				poses.push_back(lastWorldState);
				unsentPoses.push_back(lastWorldState);
				newDataForClient = true;
			}
		}
		mutex.unlock();
	}
}

void startServer(server &wsServer) {
	//void startServer(server &wsServer) starts and configures the WebSocket server.

	std::cout << "Starting WebSocket server...\t\t\t";
	//Almost all of this code is basically copy-pasted from an example. I don't understand how all of it works. Since it does work pretty much out of the 
	//box, I haven't taken the time to gain a deeper understanding. It's something I ought to do, but don't have the time right now. I've included by best
	//guesses below.
	wsServer.clear_access_channels(websocketpp::log::alevel::all); //This makes sure websocketpp isn't spamming the console with stuff I don't really need.
	wsServer.set_reuse_addr(true); //When you quit the program with control+c, the server actually stays open for a while. Eventually, it would time out, but
	                               //until it did, you couldn't run the program again (or rather, the program wouldn't work) because the port was already being
	                               //used by a running process. This allows the address to be reused, presumably by overruling the old process.
	wsServer.init_asio(); //Pretty sure this line actually starts the server.
	wsServer.set_message_handler(bind(&on_message,&wsServer,::_1,::_2)); //No idea.
	wsServer.listen(wsServerPort); //This is the port that the server listens on.
	wsServer.start_accept(); //I think it's at this point the server will accept connection requests.
	std::cout << "Done!" << std::endl;

	//This creates a new thread for the server, and detaches it. After detaching,the thread can run independently.
	std::thread wsServerThread(&server::run, &wsServer);
	wsServerThread.detach();

	return;
}
void startROS(int &argc, char** &argv) {
	//void startROS(int &argc, char** &argv) creates two ROS nodes. One listens to odometry, and one listens to base scans. It then holds until the program is killed.

	std::cout << "Starting ROS node...\t\t\t";
	ros::init(argc, argv, "drive_and_map");
	ros::NodeHandle odomNodeHandle; //Create the odometry node.
	ros::Subscriber odomSubscriber = odomNodeHandle.subscribe("odom", rosNodeQueueSize, odomCallback); //Subscribe the odometry node to the channel "odom".
	ros::NodeHandle scanNodeHandle; //Create the base scan node.
	ros::Subscriber scanSubscriber = scanNodeHandle.subscribe("base_scan", rosNodeQueueSize, scanCallback); //Subscribe the base scan node to the channel "base_scan".
	std::cout << "Done!" << std::endl;

	ros::spin(); //This is basically the same as a while(true) loop. While ros::spin isn't designed for multithreaded programs, it's ok here
	             //because we're only ever touching ROS in this thread. Or so I hope!

	return;
}

int main(int argc, char **argv) {
	//int main(int argc, char**argv) is the main function. All executed code is held here.

	std::cout << "Starting drive_and_map.cpp" << std::endl;

	server wsServer; //Remember that server is a typedef of websocketpp::server<websocketpp::config::asio>
	startServer(wsServer);
	startROS(argc, argv);

	wsServer.stop_perpetual(); //Used to help reduce errors from closing the server.
	wsServer.stop_listening(); //''

	return 0;
}
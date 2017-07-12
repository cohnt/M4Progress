#ifndef SCAN_MATCH
#define SCAN_MATCH

#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/LaserScan.h"
#include "world_state.h"

struct icpConfig {
	int minICPComparePoints;
	int maxICPLoopCount;
	double icpAverageDistanceTraveledThresholdSquared;
	int icpNoMovementCounterThreshold;
	double goodCorrespondenceThresholdSquared;
	double maximumPointMatchDistance;
	double percentChanceToMatchPoints;
};
struct icpOutput {
	std::array<std::array<double, 3>, 3> rotationMatrix;
	std::array<double, 2> translation;
	double theta;
};
struct svdOutput {
	std::array<std::array<double, 2>, 2> U;
	std::array<std::array<double, 2>, 2> S;
	std::array<std::array<double, 2>, 2> V;
};

std::vector<std::vector<int>> matchPoints(std::vector<std::vector<double>> pc1, std::vector<std::vector<double>> pc2);
svdOutput SVD(std::vector<std::vector<double>> A);
void transpose(std::vector<std::vector<double>> &m);
icpOutput runICP(std::vector<std::vector<double>> set1, std::vector<std::vector<double>> set2);
int optimizeScan(worldState &newScan, std::vector<worldState> map, icpConfig cfg);

#endif
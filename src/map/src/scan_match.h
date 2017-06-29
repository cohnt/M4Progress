#ifndef SCAN_MATCH
#define SCAN_MATCH

#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/LaserScan.h"
#include "world_state.h"

struct icpConfig {
	int minICPComparePoints;
	int maxICPLoopCount;
	float icpAverageDistanceTraveledThresholdSquared;
	int icpNoMovementCounterThreshold;
	float goodCorrespondenceThresholdSquared;
	float maximumPointMatchDistance;
};
struct icpOutput {
	std::vector<std::vector<float>> rotationMatrix;
	std::vector<float> translation;
	float theta;
};
struct svdOutput {
	std::vector<std::vector<float>> U;
	std::vector<std::vector<float>> S;
	std::vector<std::vector<float>> V;
};

std::vector<std::vector<int>> matchPoints(std::vector<std::vector<float>> pc1, std::vector<std::vector<float>> pc2);
svdOutput SVD(std::vector<std::vector<float>> A);
void transpose(std::vector<std::vector<float>> &m);
icpOutput runICP(std::vector<std::vector<float>> set1, std::vector<std::vector<float>> set2);
int optimizeScan(worldState &newScan, std::vector<worldState> map, icpConfig cfg);

#endif
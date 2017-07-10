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
};

int optimizeScan(worldState &newScan, std::vector<worldState> map, icpConfig cfg);

#endif
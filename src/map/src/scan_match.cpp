#include "scan_match.h"
#include <math.h>
#include <limits>

float distanceSquared(std::vector<float> a, std::vector<float> b);

std::vector<std::vector<int>> matchPoints(std::vector<std::vector<float>> pc1, std::vector<std::vector<float>> pc2, icpConfig cfg) {
	std::vector<std::vector<int>> pairIndexes;

	for(int i=0; i<pc2.size(); ++i) {
		if(rand() % 100 < 10) {
			float smallestSquaredDistance = FLT_MAX;
			int smallestSquaredDistanceIndex;
			for(int j=pc1.size()-1; j>=0; --j) {
				float d = distanceSquared(pc2[i], pc1[j]);
				if(d < smallestSquaredDistance) {
					smallestSquaredDistance = d;
					smallestSquaredDistanceIndex = j;
				}
				if(d < cfg.goodCorrespondenceThresholdSquared) {
					break;
				}
			}
			if(smallestSquaredDistance < cfg.maximumPointMatchDistance) {
				std::vector<int> pair = {i, smallestSquaredDistanceIndex};
				pairIndexes.push_back(pair);
			}
		}
	}
}
int optimizeScan(worldState &newScan, std::vector<worldState> map, icpConfig cfg) {
	std::vector<std::vector<float>> knownPoints;
	bool finished = false;
	int totalLoopCount;
	float iterationTotalSquaredDistance;

	int i=map.size()-1;
	while(i >= 0 && knownPoints.size() <= cfg.minICPComparePoints) {
		float walls[BASE_SCAN_MAX_NUM_POINTS][2];
		map[i].getWalls(walls);
		for(int j=0; j<BASE_SCAN_MAX_NUM_POINTS; ++j) {
			std::vector<float> wall = {walls[j][0], walls[j][1]};
			if(wall[0] != map[i].getOdometry().position.x || wall[1] != map[i].getOdometry().position.y) {
				knownPoints.push_back(wall);
			}
		}
	}

	while(!finished) {
		++totalLoopCount;
		if(totalLoopCount >= cfg.maxICPLoopCount) {
			//Failed scan
			//TODO
			break;
		}
		else {
			iterationTotalSquaredDistance = 0;
			std::vector<std::vector<float>> oldPoints;
			std::vector<std::vector<float>> newPoints;
			std::vector<std::vector<int>> pointPairsIndexes;
		}
	}

	return knownPoints.size();
}
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
svdOutput SVD(std::vector<std::vector<float>> A) {
	//???
}
icpOutput runICP(std::vector<std::vector<float>> set1, std::vector<std::vector<float>> set2) {
	std::vector<std::vector<float>> pi;
	std::vector<std::vector<float>> pi1;

	for(int i=0; i<set2.size(); ++i) {
		pi.push_back(set1[i]);
		pi1.push_back(set2[i]);
	}

	std::vector<float> p = {0, 0};
	std::vector<float> p1 = {0, 0};

	for(int i=0; i<pi.size(); ++i) {
		p[0] += pi[i][0];
		p[1] += pi[i][1];
		p1[0] += pi1[i][0];
		p1[1] += pi1[i][1];
	}
	p[0] = p[0]/pi.size();
	p[1] = p[1]/pi.size();
	p1[0] = p1[0]/pi1.size();
	p1[1] = p1[1]/pi1.size();

	std::vector<std::vector<float>> qi;
	std::vector<std::vector<float>> qi1;

	for(int i=0; i<pi.size(); ++i) {
		qi.push_back(std::vector<float>(pi[i][0]-p[0], pi[i][1]-p[1]));
		qi1.push_back(std::vector<float>(pi1[i][0]-p1[0], pi1[i][1]-p1[1]));
	}

	std::vector<std::vector<float>> H = {std::vector<float>(0, 0), std::vector<float>(0, 0)};
	for(int i=0; i<qi.size(); ++i){
		H[0][0] += qi1[i][0]*qi[i][0];
		H[0][1] += qi1[i][1]*qi[i][0];
		H[1][0] += qi1[i][0]*qi[i][1];
		H[1][1] += qi1[i][1]*qi[i][1];
	}

	std::vector<std::vector<float>> U;
	std::vector<std::vector<float>> S;
	std::vector<std::vector<float>> V;
	std::vector<std::vector<float>> UT; //Transpose U
	std::vector<std::vector<float>> VT; //Transpose V

	std::vector<std::vector<float>> rotationMatrix;
	std::vector<float> translationVector;

	for(int i=0; i<2; ++i) {
		for(int j=0; j<2; ++j) {
			//
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
			float newWalls[BASE_SCAN_MAX_NUM_POINTS][2];
			newScan.getWalls(newWalls);
			std::vector<std::vector<float>> newWallsVector;
			for(int i=0; i<BASE_SCAN_MAX_NUM_POINTS; ++i) {
				newWallsVector.push_back(std::vector<float>(newWalls[i][0], newWalls[i][1]));
			}
			pointPairsIndexes = matchPoints(knownPoints, newWallsVector, cfg);
			for(int i=0; i<pointPairsIndexes.size(); ++i) {
				oldPoints.push_back(newWallsVector[pointPairsIndexes[i][0]]);
				newPoints.push_back(newWallsVector[pointPairsIndexes[i][1]]);
			}
			icpOutput results = runICP(oldPoints, newPoints);
		}
	}

	return knownPoints.size();
}
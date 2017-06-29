#include "scan_match.h"
#include <math.h>
#include <limits>
#include <eigen3/Eigen/Dense>

float distanceSquared(std::vector<float> a, std::vector<float> b);

std::vector<std::vector<int>> matchPoints(std::vector<std::vector<float>> &pc1, std::vector<std::vector<float>> &pc2, icpConfig cfg) {
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

	return pairIndexes;
}
svdOutput SVD(std::vector<std::vector<float>> A) {
	using namespace Eigen;

	MatrixXf m(2, 2);
	m(0, 0) = A[0][0];
	m(1, 0) = A[1][0];
	m(0, 1) = A[0][1];
	m(1, 1) = A[1][1];

	JacobiSVD<MatrixXf> svd(m, ComputeThinU | ComputeThinV);

	MatrixXf Um = svd.matrixU();
	MatrixXf Vm = svd.matrixV();
	MatrixXf Id(2, 2);
	Id(0, 0) = 1; Id(0, 1) = 0; Id(1, 0) = 0; Id(1, 1) = 1;
	VectorXf Sv = svd.singularValues();
	MatrixXf Sm = Id*Sv;

	std::vector<std::vector<float>> U;
	U.push_back(std::vector<float>(Um(0, 0), Um(0, 1)));
	U.push_back(std::vector<float>(Um(1, 0), Um(1, 1)));
	std::vector<std::vector<float>> V;
	V.push_back(std::vector<float>(Vm(0, 0), Vm(0, 1)));
	V.push_back(std::vector<float>(Vm(1, 0), Vm(1, 1)));
	std::vector<std::vector<float>> S;
	S.push_back(std::vector<float>(Sm(0, 0), Sm(0, 1)));
	S.push_back(std::vector<float>(Sm(1, 0), Sm(1, 1)));

	svdOutput output;
	output.U = U;
	output.V = V;
	output.S = S;
	return output;
}
void transpose(std::vector<std::vector<float>> &m) {
	for(int i=0; i<m.size(); ++i) {
		for(int j=i; j<m[i].size(); ++j) {
			if(m[i][j] != m[j][i]) {
				m[i][j] = m[i][j] + m[j][i];
				m[j][i] = m[i][j] - m[j][i];
				m[i][j] = m[i][j] - m[j][i];
			}
		}
	}
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

	svdOutput out = SVD(H);

	std::vector<std::vector<float>> U = out.U;
	std::vector<std::vector<float>> S = out.S;
	std::vector<std::vector<float>> V = out.V;
	std::vector<std::vector<float>> UT = U; transpose(UT);
	std::vector<std::vector<float>> VT = U; transpose(VT);

	std::vector<std::vector<float>> rotationMatrix; //V*UT
	rotationMatrix.push_back(std::vector<float>((V[0][0]*UT[0][0])+(V[0][1]*UT[1][0]), (V[0][0]*UT[0][1])+(V[0][1]*UT[1][1])));
	rotationMatrix.push_back(std::vector<float>((V[1][0]*UT[0][0])+(V[1][1]*UT[1][0]), (V[1][0]*UT[0][1])+(V[1][1]*UT[1][1])));
	std::vector<float> translationVector; //p1-(rotationMatrix*p)
	translationVector.push_back(p1[0]-((rotationMatrix[0][0]*p[0])+(rotationMatrix[0][1]*p[1])));
	translationVector.push_back(p1[1]-((rotationMatrix[1][0]*p[0])+(rotationMatrix[1][1]*p[1])));

	for(int i=0; i<2; ++i) {
		for(int j=0; j<2; ++j) {
			if(rotationMatrix[i][j] > 1) {
				rotationMatrix[i][j] = 1;
			}
			else if(rotationMatrix[i][j] < -1) {
				rotationMatrix[i][j] = -1;
			}
		}
	}

	return (icpOutput){rotationMatrix, translationVector, atan2(rotationMatrix[1][0], rotationMatrix[0][0])};
}
int optimizeScan(worldState &newScan, std::vector<worldState> map, icpConfig cfg) {
	std::vector<std::vector<float>> knownPoints;
	bool finished = false;
	int totalLoopCount;
	float iterationTotalSquaredDistance;
	float iterationAverageSquaredDistance;
	std::vector<std::vector<float>> oldScanPoints;
	int icpLoopCounter = 0;
	float scanAngleError = 0;
	std::vector<float> scanPositionError = {0, 0};
	std::vector<std::vector<float>> scanTransformError = {std::vector<float>(1, 0), std::vector<float>(0, 1)};

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
			//
			// THE ERROR OCCURS WHEN YOU CALL matchPoints()!!!!!
			//
			pointPairsIndexes = matchPoints(knownPoints, newWallsVector, cfg);
			for(int i=0; i<pointPairsIndexes.size(); ++i) {
				oldPoints.push_back(newWallsVector[pointPairsIndexes[i][0]]);
				newPoints.push_back(newWallsVector[pointPairsIndexes[i][1]]);
			}
			icpOutput results = runICP(oldPoints, newPoints);
			std::vector<std::vector<float>> rotationMatrix = results.rotationMatrix;
			std::vector<float> translationVector = results.translation;
			float angle = results.theta;

			for(int i=0; i<newWallsVector.size(); ++i) {
				if(oldScanPoints.size() >= i) {
					oldScanPoints.push_back(newWallsVector[i]);
				}
				else {
					oldScanPoints[i] = newWallsVector[i];
				}
				newWallsVector[i][0] = translationVector[0] + ((rotationMatrix[0][0]*newWallsVector[i][0])+(rotationMatrix[0][1]*newWallsVector[i][1]));
				newWallsVector[i][1] = translationVector[1] + ((rotationMatrix[1][0]*newWallsVector[i][0])+(rotationMatrix[1][1]*newWallsVector[i][1]));
				iterationTotalSquaredDistance += distanceSquared(oldScanPoints[i], newWallsVector[i]);
			}
			iterationAverageSquaredDistance = iterationTotalSquaredDistance / newWallsVector.size();

			if(iterationAverageSquaredDistance < cfg.icpAverageDistanceTraveledThresholdSquared) {
				++icpLoopCounter;
				if(icpLoopCounter >= cfg.icpNoMovementCounterThreshold) {
					finished = true;
				}
			}
			else {
				icpLoopCounter = 0;
			}

			scanAngleError += angle;
			scanPositionError[0] += translationVector[0];
			scanPositionError[1] += translationVector[1];
			scanTransformError[0][0] = (scanTransformError[0][0]*rotationMatrix[0][0])+(scanTransformError[0][1]*rotationMatrix[1][0]); scanTransformError[0][1] = (scanTransformError[0][0]*rotationMatrix[0][1])+(scanTransformError[0][1]*rotationMatrix[1][1]);
			scanTransformError[1][0] = (scanTransformError[1][0]*rotationMatrix[0][0])+(scanTransformError[1][1]*rotationMatrix[1][0]); scanTransformError[1][1] = (scanTransformError[1][0]*rotationMatrix[0][1])+(scanTransformError[1][1]*rotationMatrix[1][1]);
		}
	}

	return knownPoints.size();
}
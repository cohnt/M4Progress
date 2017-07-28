#include "scan_match.h"
#include <math.h>
#include <limits>
#include <eigen3/Eigen/Dense>
#include <assert.h>

double distanceSquared(std::array<double, 3> a, std::array<double, 3> b) {
	double sum = 0;
	for(int i=0; i<a.size(); ++i) {
		sum += pow(a[i]-b[i], 2);
	}
	return sum;
}
std::array<std::array<double, 3>, 3> product(std::array<std::array<double, 3>, 3> a, std::array<std::array<double, 3>, 3> b) {
	std::array<std::array<double, 3>, 3> output = {
		std::array<double, 3>{0, 0, 0},
		std::array<double, 3>{0, 0, 0},
		std::array<double, 3>{0, 0, 0}
	};

	for(int i=0; i<a.size(); ++i) {
		for(int j=0; j<a[i].size(); ++j) {
			for(int k=0; k<b.size(); ++k) {
				output[i][j] += a[i][k] * b[k][j];
			}
		}
	}

	return output;
}

std::vector<std::array<double, 3>> matchPoints(std::vector<std::array<double, 3>> &pc1, std::vector<std::array<double, 3>> &pc2, icpConfig cfg) {
	std::vector<std::array<double, 3>> pairIndexes;

	assert(pc1.size() > 0);
	assert(pc2.size() > 0);

	std::vector<int> pc2UsedIndexes;
	const int numPointsToMatch = static_cast<int>(ceil(0.05 * static_cast<double>(pc2.size())));
	//std::cout << "Matching " << numPointsToMatch << " points..." << std::endl;
	pc2UsedIndexes.reserve(numPointsToMatch);

	while(pc2UsedIndexes.size() < numPointsToMatch) {
		int index = rand() % pc2.size();
		bool use = true;
		for(int i=0; i<pc2UsedIndexes.size(); ++i) {
			if(pc2UsedIndexes[i] == index) {
				use = false;
				break;
			}
		}
		if(use) {
			pc2UsedIndexes.push_back(index);
		}
	}

	for(int i=0; i<pc2UsedIndexes.size(); ++i) {
		const int index = pc2UsedIndexes[i];
		double smallestSquaredDistance = FLT_MAX;
		int smallestSquaredDistanceIndex;
		int j;
		for(j=pc1.size()-1; j>=0; --j) {
			double d = distanceSquared(pc2[index], pc1[j]);
			if(d < smallestSquaredDistance) {
				smallestSquaredDistance = d;
				smallestSquaredDistanceIndex = j;
			}
			if(d < cfg.goodCorrespondenceThresholdSquared) {
				break;
			}
		}
		assert(j == -1);
		std::array<double, 3> pair = {index, smallestSquaredDistanceIndex, smallestSquaredDistance};
		pairIndexes.push_back(pair);
	}

	assert(pairIndexes.size() > 0);

	return pairIndexes;
}
svdOutput SVD(std::array<std::array<double, 2>, 2> A) {
	using namespace Eigen;

	MatrixXf m(2, 2);
	m(0, 0) = A[0][0];
	m(1, 0) = A[1][0];
	m(0, 1) = A[0][1];
	m(1, 1) = A[1][1];

	BDCSVD<MatrixXf> svd(m, ComputeThinU | ComputeThinV);

	MatrixXf Um = svd.matrixU();
	MatrixXf Vm = svd.matrixV();
	MatrixXf Id(2, 2);
	Id(0, 0) = 1; Id(0, 1) = 0; Id(1, 0) = 0; Id(1, 1) = 1;
	VectorXf Sv = svd.singularValues();
	MatrixXf Sm = Id*Sv;

	std::array<std::array<double, 2>, 2> U = {
		std::array<double, 2>{Um(0, 0), Um(0, 1)},
		std::array<double, 2>{Um(1, 0), Um(1, 1)}
	};
	std::array<std::array<double, 2>, 2> V = {
		std::array<double, 2>{Vm(0, 0), Vm(0, 1)},
		std::array<double, 2>{Vm(1, 0), Vm(1, 1)}
	};
	std::array<std::array<double, 2>, 2> S = {
		std::array<double, 2>{Sv(0), 0},
		std::array<double, 2>{0, Sv(1)}
	};

	svdOutput output;
	output.U = U;
	output.V = V;
	output.S = S;
	return output;
}
void transpose(std::array<std::array<double, 2>, 2> &m) {
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
icpOutput runICP(std::vector<std::array<double, 3>> set1, std::vector<std::array<double, 3>> set2) {
	std::vector<std::array<double, 2>> pi;
	std::vector<std::array<double, 2>> pi1;

	for(int i=0; i<set2.size(); ++i) {
		pi.push_back(std::array<double, 2>{set1[i][0], set1[i][1]});
		pi1.push_back(std::array<double, 2>{set2[i][0], set2[i][1]});
	}

	std::array<double, 2> p = {0, 0};
	std::array<double, 2> p1 = {0, 0};

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

	std::vector<std::array<double, 2>> qi;
	std::vector<std::array<double, 2>> qi1;

	for(int i=0; i<pi.size(); ++i) {
		qi.push_back(std::array<double, 2>{
			pi[i][0]-p[0],
			pi[i][1]-p[1]
		});
		qi1.push_back(std::array<double, 2>{
			pi1[i][0]-p1[0],
			pi1[i][1]-p1[1]
		});
	}

	std::array<std::array<double, 2>, 2> H = {
		std::array<double, 2>{0, 0},
		std::array<double, 2>{0, 0}
	};
	for(int i=0; i<qi.size(); ++i){
		H[0][0] += qi1[i][0]*qi[i][0];
		H[0][1] += qi1[i][1]*qi[i][0];
		H[1][0] += qi1[i][0]*qi[i][1];
		H[1][1] += qi1[i][1]*qi[i][1];
	}

	svdOutput out = SVD(H);

	std::array<std::array<double, 2>, 2> U = out.U;
	std::array<std::array<double, 2>, 2> S = out.S;
	std::array<std::array<double, 2>, 2> V = out.V;
	std::array<std::array<double, 2>, 2> UT = U; transpose(UT);
	std::array<std::array<double, 2>, 2> VT = U; transpose(VT);

	std::array<std::array<double, 3>, 3> rotationMatrix = { //V*UT
		std::array<double, 3>{(V[0][0]*UT[0][0])+(V[0][1]*UT[1][0]), (V[0][0]*UT[0][1])+(V[0][1]*UT[1][1]), 0},
		std::array<double, 3>{(V[1][0]*UT[0][0])+(V[1][1]*UT[1][0]), (V[1][0]*UT[0][1])+(V[1][1]*UT[1][1]), 0},
		std::array<double, 3>{0, 0, 1}
	};
	std::array<double, 2> translationVector = {
		p1[0]-((rotationMatrix[0][0]*p[0])+(rotationMatrix[0][1]*p[1])),
		p1[1]-((rotationMatrix[1][0]*p[0])+(rotationMatrix[1][1]*p[1]))
	};

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

	return (icpOutput){rotationMatrix, translationVector, atan2(rotationMatrix[1][0], rotationMatrix[0][0]), p, p1, H, U, S, V, set1.size(), set2.size()};
}
optimizationOutput optimizeScan(worldState &newScan, std::vector<worldState> map, icpConfig cfg) {
	std::vector<std::array<double, 3>> knownPoints;
	bool finished = false;
	int totalLoopCount;
	double iterationTotalSquaredDistance;
	double iterationAverageSquaredDistance;
	std::vector<std::array<double, 3>> oldScanPoints;
	int icpLoopCounter = 0;
	double scanAngleError = 0;
	std::array<double, 2> scanPositionError = {0, 0};
	std::array<std::array<double, 3>, 3> scanTransformError = {
		std::array<double, 3>{1, 0, 0},
		std::array<double, 3>{0, 1, 0},
		std::array<double, 3>{0, 0, 1}
	};
	std::vector<std::array<double, 3>> newWallsVector = newScan.getWalls();

	int i=map.size()-1;
	knownPoints.reserve(cfg.minICPComparePoints + BASE_SCAN_MAX_NUM_POINTS);
	while(i >= 0 && knownPoints.size() <= cfg.minICPComparePoints) {
		std::vector<std::array<double, 3>> walls = map[i].getWalls();
		for(int j=0; j<walls.size(); ++j) {
			knownPoints.push_back(walls[j]);
		}
	}

	optimizationOutput output;

	totalLoopCount = 0;
	while(!finished) {
		++totalLoopCount;
		if(totalLoopCount >= cfg.maxICPLoopCount) {
			//Failed scan
			//TODO
			break;
		}
		else {
			iterationTotalSquaredDistance = 0;
			std::vector<std::array<double, 3>> oldPoints;
			std::vector<std::array<double, 3>> newPoints;
			std::vector<std::array<double, 3>> pointPairsIndexes;
			pointPairsIndexes = matchPoints(knownPoints, newWallsVector, cfg);
			assert(pointPairsIndexes.size() > 0);
			oldPoints.reserve(newWallsVector.size());
			newPoints.reserve(newWallsVector.size());
			for(int i=0; i<pointPairsIndexes.size(); ++i) {
				newPoints.push_back(newWallsVector[pointPairsIndexes[i][0]]); //retVal.push_back(newWallsVector[pointPairsIndexes[i][0]]);
				oldPoints.push_back(knownPoints[pointPairsIndexes[i][1]]);    //retVal.push_back(knownPoints[pointPairsIndexes[i][1]]);
				                                                              //retVal.push_back(std::array<double, 3>{pointPairsIndexes[i][2], 0, 0});
			}

			icpOutput results = runICP(newPoints, oldPoints);
			std::array<std::array<double, 3>, 3> rotationMatrix = results.rotationMatrix;
			std::array<double, 2> translationVector = results.translation;
			double angle = results.theta;
			//std::cout << "TRANSFORM:" << std::endl;
			//std::cout << "\tTranslation: " << translationVector[0] << ", " << translationVector[1] << std::endl;
			//std::cout << "\tRotation: " << rotationMatrix[0][0] << ", " << rotationMatrix[0][1] << std::endl;
			//std::cout << "\t          " << rotationMatrix[1][0] << ", " << rotationMatrix[1][1] << std::endl;
			//std::cout << std::endl;
			//std::cout << "H Matrix: " << results.H[0][0] << ", " << results.H[0][1] << std::endl;
			//std::cout << "          " << results.H[1][0] << ", " << results.H[1][1] << std::endl;
			//std::cout << "U Matrix: " << results.U[0][0] << ", " << results.U[0][1] << std::endl;
			//std::cout << "          " << results.U[1][0] << ", " << results.U[1][1] << std::endl;
			//std::cout << "S Matrix: " << results.S[0][0] << ", " << results.S[0][1] << std::endl;
			//std::cout << "          " << results.S[1][0] << ", " << results.S[1][1] << std::endl;
			//std::cout << "V Matrix: " << results.V[0][0] << ", " << results.V[0][1] << std::endl;
			//std::cout << "          " << results.V[1][0] << ", " << results.V[1][1] << std::endl;
			for(int i=0; i<newWallsVector.size(); ++i) {
				if(i >= oldScanPoints.size()) {
					oldScanPoints.push_back(newWallsVector[i]);
				}
				else {
					oldScanPoints[i] = newWallsVector[i];
				}
				double x = newWallsVector[i][0];
				double y = newWallsVector[i][1];
				newWallsVector[i][0] = translationVector[0] + ((rotationMatrix[0][0]*x)+(rotationMatrix[0][1]*y));
				newWallsVector[i][1] = translationVector[1] + ((rotationMatrix[1][0]*x)+(rotationMatrix[1][1]*y));
				double d2 = distanceSquared(oldScanPoints[i], newWallsVector[i]);
				iterationTotalSquaredDistance += distanceSquared(oldScanPoints[i], newWallsVector[i]);
			}
			iterationAverageSquaredDistance = iterationTotalSquaredDistance / static_cast<double>(newWallsVector.size());

			output.avgD.push_back(iterationAverageSquaredDistance);
			output.angleHistory.push_back(angle);
			output.translationHistory.push_back(translationVector);

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
			scanTransformError = product(scanTransformError, rotationMatrix);
			scanTransformError[0][2] += translationVector[0];
			scanTransformError[1][2] += translationVector[1];
		}
	}

	for(int i=0; i<newScan.walls.size(); ++i) {
		newScan.walls[i] = newWallsVector[i];
	}

	output.icpLoopCount = totalLoopCount;
	output.netAngleError = scanAngleError;
	output.netPositionError = scanPositionError;
	output.currentSLAM = scanTransformError;
	return output;
}
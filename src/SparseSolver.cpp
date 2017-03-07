/*
 * SparseSolver.cpp
 *
 *  Created on: 07/mar/2017
 *      Author: istin
 */

#include "SparseSolver.h"

using namespace std;
using namespace Eigen;

namespace optimizer {

SparseSolver::SparseSolver() {
	// TODO Auto-generated constructor stub

}

SparseSolver::~SparseSolver() {
	// TODO Auto-generated destructor stub
}

SparseSolver::SparseSolver(const RobotTrajectory& robot_poses_,
		const LandmarkPointsContainer& land_points_,
		const OdometryMeasContainer& zr_,
		const LandmarkMeasContainer& zl_,
		const float l_, const float epsilon_){
	_robot_poses = robot_poses_;
	_land_points = land_points_;
	_Zr = zr_;
	_Zl = zl_;
	_lambda = l_;
	_threshold = epsilon_;

	int problem_dim = _robot_poses.size() * _land_points.size();
}

void SparseSolver::linearizeLandmark(void){
	return;
}
void SparseSolver::linearizeOdometry(void){
	return;
}
void SparseSolver::errorAndJacobianLandmark(void){
	return;
}
void SparseSolver::errorAndJacobianOdometry(void){
	return;
}

void SparseSolver::oneStep(void){
	return;
}

} /* namespace optimizer */

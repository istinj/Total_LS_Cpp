/*
 * Solver.cpp
 *
 *  Created on: 20/feb/2017
 *      Author: istin
 */

#include "Solver.h"

Solver::Solver() {
	// TODO Auto-generated constructor stub

}

Solver::~Solver() {
	// TODO Auto-generated destructor stub
}

void Solver::init(const Matrix4fVector& robot_poses,
		const Vector3fVector& land_points,
		const Vector2fVector& proj_points,
		const Eigen::Matrix3f& camera_matrix){
	_robot_poses = robot_poses;
	_land_points = land_points;
	_proj_points = proj_points;
	_K = camera_matrix;
}

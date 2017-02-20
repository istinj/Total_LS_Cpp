/*
 * Solver.h
 *
 *  Created on: 20/feb/2017
 *      Author: istin
 */
#pragma once
#ifndef SOLVER_H_
#define SOLVER_H_

#include <Eigen/Core>
#include<Eigen/StdVector>

#include "utilities.h"

typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> Matrix4fVector;
typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> Vector3fVector;
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> Vector2fVector;


class Solver {
public:
	Solver();
	virtual ~Solver();

	void init(const Matrix4fVector& robot_poses,
			const Vector3fVector& land_points,
			const Vector2fVector& proj_points,
			const Eigen::Matrix3f& camera_matrix);

private:
	bool errorAndJacobian(void);
	void linearize(void);

	//! Solver Params
	Matrix4fVector _robot_poses;
	Vector3fVector _land_points;
	Vector2fVector _proj_points;

	Eigen::Matrix3f _K;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

#endif /* SOLVER_H_ */

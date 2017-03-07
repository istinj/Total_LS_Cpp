/*
 * SparseSolver.h
 *
 *  Created on: 07/mar/2017
 *      Author: istin
 */

#ifndef SPARSESOLVER_H_
#define SPARSESOLVER_H_

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>

#include "utilities.h"
#include "Graph.h"

namespace optimizer {

typedef Eigen::Matrix<float, 3, 6> Matrix3_6f;
typedef Eigen::Matrix<float, 2, 3> Matrix2_3f;
typedef Eigen::Matrix<float, 2, 6> Matrix2_6f;
typedef Eigen::Matrix<float, 12, 6> Matrix12_6f;
typedef Eigen::Matrix<float, 12, 1> Vector12f;

typedef std::vector<VertexSE3> RobotTrajectory;
typedef std::vector<VertexXYZ> LandmarkPointsContainer;
typedef std::vector<EdgeOdometry> OdometryMeasContainer;
typedef std::vector<EdgePosePoint> LandmarkMeasContainer;

class SparseSolver {
public:
	SparseSolver();
	SparseSolver(const RobotTrajectory& robot_poses_,
			const LandmarkPointsContainer& land_points_,
			const OdometryMeasContainer& zr_,
			const LandmarkMeasContainer& zl_,
			const float l_, const float epsilon_);
	virtual ~SparseSolver();

	void oneStep(void);

private:
	void linearizeLandmark(void);
	void linearizeOdometry(void);
	void errorAndJacobianLandmark(void);
	void errorAndJacobianOdometry(void);

	RobotTrajectory _robot_poses;
	LandmarkPointsContainer _land_points;

	OdometryMeasContainer _Zr;
	LandmarkMeasContainer _Zl;

	float _lambda = -1.0;
	float _threshold = -1.0;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

} /* namespace optimizer */

#endif /* SPARSESOLVER_H_ */

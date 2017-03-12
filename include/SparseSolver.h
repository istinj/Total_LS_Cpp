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
#include <set>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>

#include "utilities.h"
#include "Graph.h"
#include "Hessian.hpp"

namespace optimizer {
typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 3, 6> Matrix3_6f;
typedef Eigen::Matrix<float, 6, 3> Matrix6_3f;
typedef Eigen::Matrix<float, 2, 3> Matrix2_3f;
typedef Eigen::Matrix<float, 2, 6> Matrix2_6f;
typedef Eigen::Matrix<float, 12, 6> Matrix12_6f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;
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
	bool linearizeLandmark(float& total_chi_, int& inliers_);
	bool linearizeOdometry(float& total_chi_, int& inliers_);
	void errorAndJacobianLandmark(const RobotPose& xr,
			const LandmarkXYZ& xl,
			const LandmarkMeas& zl,
			Eigen::Vector3f& error,
			Eigen::Matrix3f& Jl,
			Matrix3_6f& Jr);
	void errorAndJacobianOdometry(const RobotPose& xi,
			const RobotPose& xj,
			const OdometryMeas& zr,
			Vector12f& error,
			Matrix12_6f& Ji,
			Matrix12_6f& Jj);

	int getPoseMatrixIndex(int curr_pose_idx);
	int getLandMatrixIndex(int curr_land_idx);

	RobotTrajectory _robot_poses;
	LandmarkPointsContainer _land_points;

	OdometryMeasContainer _Zr;
	LandmarkMeasContainer _Zl;

	//! NB no matrixXf -> no cached operations
	//! Hashmap (<index i, index j>, *_DataType)
	//! hashmap di puntatori a data type
	std::set<GenericHessian*> _HessianContainer;
	//! TODO Remember to clean-up everything in the destructor (or at the end of the iteration)
	//! TODO Same for the RHSVector;

	float _lambda = -1.0;
	float _threshold = -1.0;


public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

} /* namespace optimizer */

#endif /* SPARSESOLVER_H_ */

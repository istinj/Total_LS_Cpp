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
#include <boost/unordered_map.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>

#include "utilities.h"
#include "Graph.h"

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

//! TODO no matrixXf -> no cached operations
//! Hashmap (<index i, index j>, *_DataType)
typedef std::map<std::pair<int, int>, Eigen::MatrixXf> HessianContainer;
typedef std::map<int, Eigen::MatrixXf> RHSContainer;

struct HessianBlock {
	int i_idx;
	int j_idx;
	Eigen::MatrixXf data;
};

struct RHSBlock {
	int idx;
	Eigen::VectorXf data;
};

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

	template<typename _MatrixType>
	void addHessianBlock(const std::pair<int, int>& hessian_indices_,
			const _MatrixType& hessian_block_);

	template<typename _VectorType>
	void addRHSBlock(const int rhs_index_,
			const _VectorType& rhs_block_);

	RobotTrajectory _robot_poses;
	LandmarkPointsContainer _land_points;

	OdometryMeasContainer _Zr;
	LandmarkMeasContainer _Zl;

	//hashmap di puntatori a data type
	HessianContainer _H_container;
	RHSContainer _b_container;
	std::vector<HessianBlock> _H;
	std::vector<RHSBlock> _b;

	float _lambda = -1.0;
	float _threshold = -1.0;


public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

} /* namespace optimizer */

#endif /* SPARSESOLVER_H_ */

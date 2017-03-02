/*
 * Solver.h
 *
 *  Created on: 20/feb/2017
 *      Author: istin
 */
#pragma once
#ifndef SOLVER_H_
#define SOLVER_H_
#define IMG_COLS 540
#define IMG_ROWS 480
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>

#include "utilities.h"

typedef Eigen::Matrix<float, 3, 6> Matrix3_6f;
typedef Eigen::Matrix<float, 2, 3> Matrix2_3f;
typedef Eigen::Matrix<float, 2, 6> Matrix2_6f;
typedef Eigen::Matrix<float, 12, 6> Matrix12_6f;
typedef Eigen::Matrix<float, 12, 1> Vector12f;
typedef std::vector<Eigen::Matrix4f> Matrix4fVector;
typedef std::vector<Eigen::Vector3f> Vector3fVector;
typedef std::vector<Eigen::Vector2f> Vector2fVector;

struct Stats{
	std::vector<float> l_chis; std::vector<int> l_inliers;
	std::vector<float> p_chis; std::vector<int> p_inliers;
	std::vector<float> r_chis; std::vector<int> r_inliers;
};


class Solver {
public:
	Solver();
	virtual ~Solver();

	void init(const Matrix4fVector& robot_poses,
			const Vector3fVector& land_points,
			const Vector3fVector& land_meas,
			const Vector2fVector& proj_meas,
			const Matrix4fVector& odom_meas,
			const std::vector<Association>& land_association,
			const std::vector<Association>& proj_association,
			const std::vector<Association>& odom_association,
			const Eigen::Matrix3f& camera_matrix,
			const float lambda);

	void doIterations(const int iterations,
			Matrix4fVector& new_robot_poses,
			Vector3fVector& new_landmark_points,
			Stats& final_stats);

private:
	//! Landmark
	void linearizeLandmarks(Eigen::MatrixXf& out_H,
			Eigen::VectorXf& out_b, float& out_chi, int& out_inliers);
	void errorAndJacobianLand(const Eigen::Matrix4f& xr,
			const Eigen::Vector3f& xl,
			const Eigen::Vector3f& z,
			Eigen::Vector3f& error,
			Eigen::Matrix3f& JL,
			Matrix3_6f& JR);

	//! Projection
	void linearizeProjections(Eigen::MatrixXf& out_H,
			Eigen::VectorXf& out_b, float& out_chi, int& inliers);
	bool errorAndJacobianProj(const Eigen::Matrix4f& xr,
			const Eigen::Vector3f& xl,
			const Eigen::Vector2f& z,
			Eigen::Vector2f& error,
			Matrix2_3f& JL,
			Matrix2_6f& JR);

	//! Odometry
	void linearizeOdometry(Eigen::MatrixXf& out_H,
			Eigen::VectorXf& out_b, float& out_chi, int& inliers);
	void errorAndJacobianOdometry(const Eigen::Matrix4f& xi,
			const Eigen::Matrix4f& xj,
			const Eigen::Matrix4f& z,
			Vector12f& error,
			Matrix12_6f& JI,
			Matrix12_6f& JJ);


	void boxPlus(const Eigen::VectorXf& dX, Matrix4fVector& robot_poses, Vector3fVector& land_points);
	int getPoseMatrixIndex(int curr_pose_idx);
	int getLandMatrixIndex(int curr_land_idx);

	//! Solver Params
	Matrix4fVector _robot_poses;
	Vector3fVector _land_points;

	Vector3fVector _zl;
	Vector2fVector _zp;
	Matrix4fVector _zr;

	std::vector<Association> _l_assoc;
	std::vector<Association> _p_assoc;
	std::vector<Association> _r_assoc;

	Eigen::Matrix3f _K;

	const int _x_dim = 6;
	const int _l_dim = 3;
	int _problem_dim = -1;
	float _threshold = 500.0;
	float _threshold_proj = 500.0;
	float _lambda = 0.0;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

#endif /* SOLVER_H_ */

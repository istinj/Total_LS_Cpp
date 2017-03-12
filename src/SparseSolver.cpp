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

bool SparseSolver::linearizeLandmark(float& total_chi_, int& inliers_){
	Matrix3f Jl = Matrix3f::Zero();
	Matrix3_6f Jr = Matrix3_6f::Zero();
	Vector3f e = Vector3f::Zero();

	Matrix6f h_pp;
	Matrix6_3f h_pl;
	Matrix3_6f h_lp;
	Matrix3f h_ll;
	Vector6f b_pose;
	Vector3f b_land;

	total_chi_= 0.0;
	inliers_ = 0;

	for(LandmarkMeasContainer::iterator it = _Zl.begin(); it != _Zl.end(); ++it){
		std::pair<int, int> curr_association = it->association();

		RobotTrajectory::iterator pose_iter = std::find(_robot_poses.begin(),
				_robot_poses.end(), curr_association.first);
		if(pose_iter == _robot_poses.end()){
			cerr << "Error, bad data association" << endl;
			return false;
		}
		LandmarkPointsContainer::iterator land_iter = std::find(_land_points.begin(),
				_land_points.end(), curr_association.second);
		if(land_iter == _land_points.end()){
			cerr << "Error, bad data association" << endl;
			return false;
		}
		errorAndJacobianLandmark(pose_iter->data(),
				land_iter->data(),
				it->data(),
				e, Jl, Jr);

		float chi = e.transpose() * e;
		if(chi > _threshold){
			e *= sqrt(_threshold/chi);
			chi = _threshold;
		} else {
			inliers_++;
		}
		total_chi_ += chi;


		//! TODO H?
		int row_idx_hessian = getPoseMatrixIndex(pose_iter->index());
		int col_idx_hessian = getPoseMatrixIndex(land_iter->index());

		h_pp = Jr.transpose() * Jr;
		pair<int, int> hessian_indices = make_pair(row_idx_hessian,
				row_idx_hessian);
		addHessianBlock(hessian_indices, h_pp);

		h_pl = Jr.transpose() * Jl;
		hessian_indices = make_pair(row_idx_hessian,
				col_idx_hessian);
		addHessianBlock(hessian_indices, h_pl);

		h_lp = Jl.transpose() * Jr;
		hessian_indices = make_pair(col_idx_hessian,
				row_idx_hessian);
		addHessianBlock(hessian_indices, h_lp);

		h_ll = Jl.transpose() * Jl;
		hessian_indices = make_pair(col_idx_hessian,
				col_idx_hessian);
		addHessianBlock(hessian_indices, h_ll);

		//! TODO B?
		b_pose = Jr.transpose() * e;
		addRHSBlock(row_idx_hessian, b_pose);

		b_land = Jl.transpose() * e;
		addRHSBlock(col_idx_hessian, b_land);

		/**/

		//! TODO B?
	}

	return true;
}

bool SparseSolver::linearizeOdometry(float& total_chi_, int& inliers_) {
	total_chi_ = 0.0;
	inliers_ = 0;

	Matrix12_6f Ji = Matrix12_6f::Zero();
	Matrix12_6f Jj = Matrix12_6f::Zero();
	Vector12f e = Vector12f::Zero();

	Matrix<float, 6, 6> h_ii, h_ij, h_ji, h_jj;
	Matrix<float, 6, 1> b_i, b_j;

	Matrix<float, 12, 12> Omega = Matrix<float, 12, 12>::Identity();
	Omega.block<9,9>(0,0) *= 1000.0;

	for (OdometryMeasContainer::iterator it = _Zr.begin(); it != _Zr.end();	++it) {
		std::pair<int, int> curr_association = it->association();

		RobotTrajectory::iterator pose_i_iter = std::find(_robot_poses.begin(),
				_robot_poses.end(), curr_association.first);
		if(pose_i_iter == _robot_poses.end()){
			cerr << "Error, bad data association" << endl;
			return false;
		}

		RobotTrajectory::iterator pose_j_iter = std::find(_robot_poses.begin(),
				_robot_poses.end(), curr_association.second);
		if(pose_j_iter == _robot_poses.end()){
			cerr << "Error, bad data association" << endl;
			return false;
		}

		errorAndJacobianOdometry(pose_i_iter->data(),
				pose_j_iter->data(),
				it->data(),
				e, Ji, Jj);

		float chi = e.transpose() * e;
		if (chi > _threshold) {
			e *= sqrt(_threshold / chi);
			chi = _threshold;
		} else {
			inliers_++;
		}
		total_chi_ += chi;

		//! TODO H?
		int row_idx_hessian = getPoseMatrixIndex(pose_i_iter->index());
		int col_idx_hessian = getPoseMatrixIndex(pose_j_iter->index());

		h_ii = Ji.transpose() * Omega * Ji;
		pair<int, int> hessian_indices = make_pair(row_idx_hessian,
				row_idx_hessian);
		addHessianBlock(hessian_indices, h_ii);

		h_ij = Ji.transpose() * Omega * Jj;
		hessian_indices = make_pair(row_idx_hessian,
				col_idx_hessian);
		addHessianBlock(hessian_indices, h_ij);

		h_ji = Jj.transpose() * Omega * Ji;
		hessian_indices = make_pair(col_idx_hessian,
				row_idx_hessian);
		addHessianBlock(hessian_indices, h_ji);

		h_jj = Jj.transpose() * Omega * Jj;
		hessian_indices = make_pair(col_idx_hessian,
				col_idx_hessian);
		addHessianBlock(hessian_indices, h_jj);


		//! TODO B?
		b_i = Ji.transpose() * Omega * e;
		addRHSBlock(row_idx_hessian, b_i);

		b_j = Jj.transpose() * Omega * e;
		addRHSBlock(col_idx_hessian, b_j);
	}

	return true;
}
void SparseSolver::errorAndJacobianLandmark(const RobotPose& xr,
		const LandmarkXYZ& xl,
		const LandmarkMeas& zl,
		Eigen::Vector3f& error,
		Eigen::Matrix3f& Jl,
		Matrix3_6f& Jr){
	Vector3f h_x = xr.linear() * xl + xr.translation();

	error = h_x - zl;

	Jl = xr.linear();
	Jr.block<3,3>(0,0).setIdentity();
	Jr.block<3,3>(0,3) = -skew(h_x);
}
void SparseSolver::errorAndJacobianOdometry(const RobotPose& xi,
		const RobotPose& xj,
		const OdometryMeas& zr,
		Vector12f& error,
		Matrix12_6f& Ji,
		Matrix12_6f& Jj){
	Matrix3f Rx0, Ry0, Rz0;
	Rx0 << 0,0,0,  0,0,-1,  0,1,0;
	Ry0 << 0,0,1,  0,0,0,   -1,0,0;
	Rz0 << 0,-1,0, 1,0,0,   0,0,0;

	Matrix3f Ri = xi.linear();
	Matrix3f Rj = xj.linear();
	Vector3f ti = xi.translation();
	Vector3f tj = xj.translation();
	Vector3f t_ij = tj-ti;

	Matrix3f dR_x = Ri.transpose() * Rx0 * Rj;
	Matrix3f dR_y = Ri.transpose() * Ry0 * Rj;
	Matrix3f dR_z = Ri.transpose() * Rz0 * Rj;

	Matrix<float, 9, 1> dr_x_flattened, dr_y_flattened, dr_z_flattened;
	dr_x_flattened << dR_x.col(0), dR_x.col(1), dR_x.col(2);
	dr_y_flattened << dR_y.col(0), dR_y.col(1), dR_y.col(2);
	dr_z_flattened << dR_z.col(0), dR_z.col(1), dR_z.col(2);

	//! Fill Jj
	Jj.block<9,1>(0,3) = dr_x_flattened;
	Jj.block<9,1>(0,4) = dr_y_flattened;
	Jj.block<9,1>(0,5) = dr_z_flattened;
	Jj.block<3,3>(9,0) = Ri.transpose();
	Jj.block<3,3>(9,3) = -Ri.transpose() * skew(tj);

	Ji = -Jj;

	RobotPose h_x = RobotPose::Identity();
	h_x.linear() = Ri.transpose() * Rj;
	h_x.translation() = Ri.transpose() * t_ij;

	//! Compose e
	Eigen::Isometry3f temp_e;
	temp_e.matrix() = h_x.matrix() - zr.matrix();

	error.block<3,1>(0,0) = temp_e.matrix().block<3,1>(0,0);
	error.block<3,1>(3,0) = temp_e.matrix().block<3,1>(0,1);
	error.block<3,1>(6,0) = temp_e.matrix().block<3,1>(0,2);
	error.block<3,1>(9,0) = temp_e.matrix().block<3,1>(0,3);
}

void SparseSolver::oneStep(void){
	float step_chi;
	int step_inliers;

	if(linearizeLandmark(step_chi,step_inliers))
		cout << CYAN << "inliers land = " << step_inliers << "\t" << "chi land = " << step_chi << RESET << endl;
	if(linearizeOdometry(step_chi,step_inliers))
		cout << GREEN << "inliers odom = " << step_inliers << "\t" << "chi odom = " << step_chi << RESET << endl;

	//	for(HessianContainer::iterator it = _H_container.begin(); it != _H_container.end(); ++it){
	//		cout << it->second << endl;
	//		cin.get();
	//	}


	return; //placeholder
}


int SparseSolver::getPoseMatrixIndex(int curr_pose_idx){
	if(curr_pose_idx > _robot_poses.size() - 1){
		cerr << "Exceeding index\nExit" << endl;
		return -1;
	}
	return curr_pose_idx * X_DIM;
}

int SparseSolver::getLandMatrixIndex(int curr_land_idx){
	if(curr_land_idx > _land_points.size() - 1){
		cerr << "Exceeding index\nExit" << endl;
		return -1;
	}
	return _robot_poses.size() * X_DIM + curr_land_idx * L_DIM;
}

template<typename _MatrixType>
void SparseSolver::addHessianBlock(const std::pair<int, int>& hessian_indices_,
		const _MatrixType& hessian_block_){
	if(_H_container[hessian_indices_].isZero()){
		_H_container[hessian_indices_].resize(hessian_block_.rows(), hessian_block_.cols());
		_H_container[hessian_indices_] = hessian_block_;
	} else {
		_H_container[hessian_indices_].noalias() += hessian_block_;
	}
}

template<typename _VectorType>
void SparseSolver::addRHSBlock(const int rhs_index_,
		const _VectorType& rhs_block_){
	if(_b_container[rhs_index_].isZero()){
		_b_container[rhs_index_].resize(rhs_block_.rows(), 1);
		_b_container[rhs_index_] = rhs_block_;
	} else {
		_b_container[rhs_index_].noalias() += rhs_block_;
	}
}

} /* namespace optimizer */

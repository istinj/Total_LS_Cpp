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

bool SparseSolver::linearizePosePoint(float& total_chi_, int& inliers_){
	Matrix3f Jl = Matrix3f::Zero();
	Matrix3_6f Jr = Matrix3_6f::Zero();
	Vector3f e = Vector3f::Zero();

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
		errorAndJacobianPosePoint(pose_iter->data(),
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

		Matrix6f h_pp_data = Jr.transpose() * Jr;
		pair<int, int> hessian_indices = make_pair(row_idx_hessian,
				row_idx_hessian);
		Hessian<Matrix6f>* H_pp = new Hessian<Matrix6f>(hessian_indices,
				h_pp_data, std::make_pair(6,6));
		_HessianContainer.insert(H_pp);

		Matrix6_3f h_pl_data = Jr.transpose() * Jl;
		hessian_indices = make_pair(row_idx_hessian,
				col_idx_hessian);
		Hessian<Matrix6_3f>* H_pl = new Hessian<Matrix6_3f>(hessian_indices,
				h_pl_data, std::make_pair(6,3));
		_HessianContainer.insert(H_pl);

		Matrix3_6f h_lp_data = Jl.transpose() * Jr;
		hessian_indices = make_pair(col_idx_hessian,
				row_idx_hessian);
		Hessian<Matrix3_6f>* H_lp = new Hessian<Matrix3_6f>(hessian_indices,
				h_lp_data, std::make_pair(3,6));
		_HessianContainer.insert(H_lp);

		Matrix3f h_ll_data = Jl.transpose() * Jl;
		hessian_indices = make_pair(col_idx_hessian,
				col_idx_hessian);
		Hessian<Matrix3f>* H_ll = new Hessian<Matrix3f>(hessian_indices,
				h_ll_data, std::make_pair(3,3));
		_HessianContainer.insert(H_ll);

		//! TODO B?
/*
		RHSBlock<Vector6f> b_block_pose;
		b_block_pose.data = Jr.transpose() * e;
		b_block_pose.blockIndex = row_idx_hessian;

		RHSBlock<Vector3f> b_block_land;
		b_block_land.data = Jl.transpose() * e;
		b_block_land.blockIndex = col_idx_hessian;
/**/
	}
	return true;
}

bool SparseSolver::linearizePosePose(float& total_chi_, int& inliers_) {
	total_chi_ = 0.0;
	inliers_ = 0;

	Matrix12_6f Ji = Matrix12_6f::Zero();
	Matrix12_6f Jj = Matrix12_6f::Zero();
	Vector12f e = Vector12f::Zero();

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

		errorAndJacobianPosePose(pose_i_iter->data(),
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

		int row_idx_hessian = getPoseMatrixIndex(pose_i_iter->index());
		int col_idx_hessian = getPoseMatrixIndex(pose_j_iter->index());

		Matrix6f h_block_data = Ji.transpose() * Omega * Ji;
		pair<int, int> hessian_indices = make_pair(row_idx_hessian,
				row_idx_hessian);
		Hessian<Matrix6f>* H_ii = new Hessian<Matrix6f>(hessian_indices,
				h_block_data, std::make_pair(6,6));

//		if(_HessianContainer.find(H_ii) != _HessianContainer.end()){
//			//! TODO sum
//		} else {
//			_HessianContainer.insert(H_ii);
//		}

		h_block_data = Ji.transpose() * Omega * Jj;
		hessian_indices = make_pair(row_idx_hessian,
				col_idx_hessian);
		Hessian<Matrix6f>* H_ij = new Hessian<Matrix6f>(hessian_indices,
				h_block_data, std::make_pair(6,6));

//		_HessianContainer.insert(H_ij);

		h_block_data = Jj.transpose() * Omega * Jj;
		hessian_indices = make_pair(col_idx_hessian,
				col_idx_hessian);
		Hessian<Matrix6f>* H_jj = new Hessian<Matrix6f>(hessian_indices,
				h_block_data, std::make_pair(6,6));

//		_HessianContainer.insert(H_jj);

		//! TODO B?
/*
		RHSBlock<Vector6f> b_block;

		b_block.data = Ji.transpose() * Omega * e;
		b_block.blockIndex = row_idx_hessian;

		b_block.data = Jj.transpose() * Omega * e;
		b_block.blockIndex = col_idx_hessian;
/**/
	}
	cout << _HessianContainer.size() << endl;
	//! Now decompose the Hessian
	bool result = CHDecompose();

	return true;
}
void SparseSolver::errorAndJacobianPosePoint(const RobotPose& xr,
		const LandmarkXYZ& xl,
		const PointMeas& zl,
		Eigen::Vector3f& error,
		Eigen::Matrix3f& Jl,
		Matrix3_6f& Jr){
	Vector3f h_x = xr.linear() * xl + xr.translation();

	error = h_x - zl;

	Jl = xr.linear();
	Jr.block<3,3>(0,0).setIdentity();
	Jr.block<3,3>(0,3) = -skew(h_x);
}
void SparseSolver::errorAndJacobianPosePose(const RobotPose& xi,
		const RobotPose& xj,
		const PoseMeas& zr,
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

	if(linearizePosePose(step_chi,step_inliers))
		cout << GREEN << "inliers odom = " << step_inliers << "\t" << "chi odom = " << step_chi << RESET << endl;

	//! TODO CLEAN-UP EVERYTHING (containers + hash_map and so on)
	for(std::set<GenericHessian*>::iterator it = _HessianContainer.begin(); it != _HessianContainer.end(); ++it){
		delete (*it);
	}
	return; //placeholder
}

bool SparseSolver::CHDecompose(void){
	for(std::set<GenericHessian*>::iterator it = _HessianContainer.begin(); it != _HessianContainer.end(); ++it){

		//! U11
		Hessian<Matrix6f>* curr_hessian_block = (Hessian<Matrix6f>*)(*it);
		Matrix6f U11;
		cholesky(curr_hessian_block->getData(), U11);

		BlockIndices block_indices = curr_hessian_block->getIndices();
		Hessian<Matrix6f>* chol_block_U11 = new Hessian<Matrix6f>(block_indices, U11, make_pair(6,6));
		_CholeskyContainer.emplace(block_indices,chol_block_U11);

		//! U12
		std::advance(it, 1);
		if(it == _HessianContainer.end())
			break;
		curr_hessian_block = (Hessian<Matrix6f>*)(*it);
		Matrix6f U11_inverse_transp = U11.transpose();
		U11_inverse_transp = U11_inverse_transp.inverse();
		Matrix6f U12 = U11_inverse_transp * curr_hessian_block->getData();
		Hessian<Matrix6f>* chol_block_U12 = new Hessian<Matrix6f>(block_indices, U11, make_pair(6,6));
		_CholeskyContainer.emplace(block_indices,chol_block_U12);

		//! TODO: Update of complement??????
		std::advance(it, 1);
		if(it == _HessianContainer.end())
			break;
		Matrix6f update_matrix = U12.transpose() * U12;
		for(std::set<GenericHessian*>::iterator update_it = it; update_it != _HessianContainer.end(); ++update_it){
			Hessian<Matrix6f>* next_hessian_block = (Hessian<Matrix6f>*)(*update_it);
			Matrix6f new_data = next_hessian_block->getData() - update_matrix;
			cout << new_data << endl;
			cin.get();
			next_hessian_block->setData(new_data);
//			_HessianContainer.erase(update_it);
//			_HessianContainer.insert(next_hessian_block);
		}

	}
//	for(std::set<GenericHessian*>::iterator it = _HessianContainer.begin(); it != _HessianContainer.end(); ++it){
//		(*it)->print();
//		cin.get();
//	}
	return true;
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

} /* namespace optimizer */

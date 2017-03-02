/*
 * Solver.cpp
 *
 *  Created on: 20/feb/2017
 *      Author: istin
 */

#include "Solver.h"
using namespace std;
using namespace Eigen;

Solver::Solver() {
	// TODO Auto-generated constructor stub

}

Solver::~Solver() {
	// TODO Auto-generated destructor stub
}

void Solver::init(const Matrix4fVector& robot_poses,
		const Vector3fVector& land_points,
		const Vector3fVector& land_meas,
		const Vector2fVector& proj_meas,
		const Matrix4fVector& odom_meas,
		const std::vector<Association>& land_association,
		const std::vector<Association>& proj_association,
		const std::vector<Association>& odom_association,
		const Eigen::Matrix3f& camera_matrix,
		const float lambda){
	_robot_poses = robot_poses;
	_land_points = land_points;

	_zl = land_meas;
	_zp = proj_meas;
	_zr = odom_meas;

	_l_assoc = land_association;
	_p_assoc = proj_association;
	_r_assoc = odom_association;

	_K = camera_matrix;
	_problem_dim = _x_dim * _robot_poses.size() + _l_dim * _land_points.size();

	_lambda = lambda;
}

//! ---------------------- LANDMARKS --------------------- //
void Solver::linearizeLandmarks(Eigen::MatrixXf& out_H,
		Eigen::VectorXf& out_b, float& out_chi, int& inliers){
	Vector3f e;
	Matrix3f Jl;
	Matrix3_6f Jr;
	inliers = 0;
	out_chi = 0;

	for(int meas_idx = 0; meas_idx < _zl.size(); meas_idx++){
		int pose_idx = _l_assoc[meas_idx].x_idx;
		int land_idx = _l_assoc[meas_idx].h_idx;
		Matrix4f Xr = _robot_poses[pose_idx];
		Vector3f Xl = _land_points[land_idx];
		Vector3f z = _zl[meas_idx];

		errorAndJacobianLand(Xr, Xl, z, e, Jl, Jr);
		float chi = e.transpose() * e;
		if(chi > _threshold){
			e *= sqrt(_threshold/chi);
			chi = _threshold;
		} else {
			inliers++;
		}
		out_chi+=chi;

		int r_matrix_idx = getPoseMatrixIndex(pose_idx);
		int l_matrix_idx = getLandMatrixIndex(land_idx);

		out_H.block<6,6>(r_matrix_idx, r_matrix_idx).noalias() += Jr.transpose() * Jr;
		out_H.block<6,3>(r_matrix_idx, l_matrix_idx).noalias() += Jr.transpose() * Jl;
		out_H.block<3,6>(l_matrix_idx, r_matrix_idx).noalias() += Jl.transpose() * Jr;
		out_H.block<3,3>(l_matrix_idx, l_matrix_idx).noalias() += Jl.transpose() * Jl;

		out_b.block<6,1>(r_matrix_idx, 0).noalias() += Jr.transpose() * e;
		out_b.block<3,1>(l_matrix_idx, 0).noalias() += Jl.transpose() * e;
	}
	return;
}

void Solver::errorAndJacobianLand(const Eigen::Matrix4f& xr,
		const Eigen::Vector3f& xl,
		const Eigen::Vector3f& z,
		Eigen::Vector3f& error,
		Eigen::Matrix3f& JL,
		Matrix3_6f& JR){
	Vector3f h_x = xr.block<3,3>(0,0) * xl + xr.block<3,1>(0,3); //! prediction
	error = h_x - z;
	JL = xr.block<3,3>(0,0);
	JR.block<3,3>(0,0).setIdentity();
	JR.block<3,3>(0,3) = -skew(h_x);
}

//! ---------------------- PROJECTIONS --------------------- //
void Solver::linearizeProjections(Eigen::MatrixXf& out_H,
		Eigen::VectorXf& out_b, float& out_chi, int& inliers){
	Vector2f e;
	Matrix2_3f Jl;
	Matrix2_6f Jr;
	inliers = 0;
	out_chi = 0;

	for(int meas_idx = 0; meas_idx < _zp.size(); meas_idx++){
		float chi = 0;
		int pose_idx = _p_assoc[meas_idx].x_idx;
		int land_idx = _p_assoc[meas_idx].h_idx;
		Matrix4f Xr = _robot_poses[pose_idx];
		Vector3f Xl = _land_points[land_idx];
		Vector2f z = _zp[meas_idx];
		if(errorAndJacobianProj(Xr, Xl, z, e, Jl, Jr)){
			chi = e.transpose() * e;
		} else {
			continue;
		}
		if(chi > _threshold_proj){
			e *= sqrt(_threshold_proj/chi);
			chi = _threshold_proj;
		} else {
			inliers++;
		}
		out_chi+=chi;

		int r_matrix_idx = getPoseMatrixIndex(pose_idx);
		int l_matrix_idx = getLandMatrixIndex(land_idx);

		out_H.block<6,6>(r_matrix_idx, r_matrix_idx).noalias() += Jr.transpose() * Jr;
		out_H.block<6,3>(r_matrix_idx, l_matrix_idx).noalias() += Jr.transpose() * Jl;
		out_H.block<3,6>(l_matrix_idx, r_matrix_idx).noalias() += Jl.transpose() * Jr;
		out_H.block<3,3>(l_matrix_idx, l_matrix_idx).noalias() += Jl.transpose() * Jl;

		out_b.block<6,1>(r_matrix_idx, 0).noalias() += Jr.transpose() * e;
		out_b.block<3,1>(l_matrix_idx, 0).noalias() += Jl.transpose() * e;
	}
	return;
}

bool Solver::errorAndJacobianProj(const Eigen::Matrix4f& xr,
		const Eigen::Vector3f& xl,
		const Eigen::Vector2f& z,
		Eigen::Vector2f& error,
		Matrix2_3f& JL,
		Matrix2_6f& JR){
	Vector3f tp = xr.block<3,3>(0,0) * xl + xr.block<3,1>(0,3); //! world coords
	//! if the point is behind the camera just drop it
	if(tp.z() < 0)
		return false;

	Vector3f pp = _K * tp;
	if(pp.z() <= 0)
		return false;

	float iz = 1./pp.z();
	Vector2f h_x;
	h_x << pp.x() * iz, pp.y() * iz;

	//! If the projection is outside the image just drop the point
	if(h_x.x() < 0 || h_x.x() > IMG_COLS ||
			h_x.y() < 0 || h_x.y() > IMG_ROWS){
		return false;
	}

	float iz2 = (float)iz * iz;

	//! Fuckin Chain Rule for the Final Jacobians
	Matrix2_3f Jp;
	Jp << iz, 0, -pp.x()*iz2,
			0, iz, -pp.y()*iz2;
	Matrix3_6f Jwr;
	Jwr.block<3,3>(0,0).setIdentity();
	Jwr.block<3,3>(0,3) = -skew(tp);
	Matrix3f Jwl = (Matrix3f)xr.block<3,3>(0,0);
	error = h_x - z;
	JL = Jp * _K * Jwl;
	JR = Jp * _K * Jwr;
	return true;
}

//! ---------------------- ODOMETRY --------------------- //
void Solver::linearizeOdometry(Eigen::MatrixXf& out_H,
		Eigen::VectorXf& out_b, float& out_chi, int& inliers){
	Vector12f e = Vector12f::Zero();
	Matrix12_6f Ji = Matrix12_6f::Zero();
	Matrix12_6f Jj = Matrix12_6f::Zero();
	out_chi = 0;
	inliers = 0;
	for(int meas_idx = 0; meas_idx < _zr.size(); meas_idx++){
		int pose_i_idx = _r_assoc[meas_idx].x_idx;
		int pose_j_idx = _r_assoc[meas_idx].h_idx;
		Matrix4f Xi = _robot_poses[pose_i_idx];
		Matrix4f Xj = _robot_poses[pose_j_idx];
		Matrix4f Z = _zr[meas_idx];

		Matrix<float, 12, 12> Omega = Matrix<float, 12, 12>::Identity();
		Omega.block<9,9>(0,0) *= 1000.0;

		errorAndJacobianOdometry(Xi, Xj, Z, e, Ji, Jj);
		float chi = e.transpose() * Omega * e;
		if(chi > _threshold){
			Omega *= sqrt(_threshold/chi);
			chi = _threshold;
		} else {
			inliers++;
		}
		out_chi+=chi;

		int pose_i_matrix_idx = getPoseMatrixIndex(pose_i_idx);
		int pose_j_matrix_idx = getPoseMatrixIndex(pose_j_idx);

		out_H.block<6,6>(pose_i_matrix_idx, pose_i_matrix_idx).noalias() += Ji.transpose() * Omega * Ji;
		out_H.block<6,6>(pose_i_matrix_idx, pose_j_matrix_idx).noalias() += Ji.transpose() * Omega * Jj;
		out_H.block<6,6>(pose_j_matrix_idx, pose_i_matrix_idx).noalias() += Jj.transpose() * Omega * Ji;
		out_H.block<6,6>(pose_j_matrix_idx, pose_j_matrix_idx).noalias() += Jj.transpose() * Omega * Jj;

		out_b.block<6,1>(pose_i_matrix_idx, 0).noalias() += Ji.transpose() * Omega * e;
		out_b.block<6,1>(pose_j_matrix_idx, 0).noalias() += Jj.transpose() * Omega * e;
	}
	return;
}

void Solver::errorAndJacobianOdometry(const Eigen::Matrix4f& xi,
		const Eigen::Matrix4f& xj,
		const Eigen::Matrix4f& z,
		Vector12f& error,
		Matrix12_6f& JI,
		Matrix12_6f& JJ){
	Matrix3f Rx0, Ry0, Rz0;
	Rx0 << 0,0,0,  0,0,-1,  0,1,0;
	Ry0 << 0,0,1,  0,0,0,   -1,0,0;
	Rz0 << 0,-1,0, 1,0,0,   0,0,0;

	Matrix3f Ri = xi.block<3,3>(0,0);
	Matrix3f Rj = xj.block<3,3>(0,0);
	Vector3f ti = xi.block<3,1>(0,3);
	Vector3f tj = xj.block<3,1>(0,3);
	Vector3f t_ij = tj-ti;

	Matrix3f dR_x = Ri.transpose() * Rx0 * Rj;
	Matrix3f dR_y = Ri.transpose() * Ry0 * Rj;
	Matrix3f dR_z = Ri.transpose() * Rz0 * Rj;

	Matrix<float, 9, 1> dr_x_flattened, dr_y_flattened, dr_z_flattened;
	dr_x_flattened << dR_x.col(0), dR_x.col(1), dR_x.col(2);
	dr_y_flattened << dR_y.col(0), dR_y.col(1), dR_y.col(2);
	dr_z_flattened << dR_z.col(0), dR_z.col(1), dR_z.col(2);

	//! Fill JJ
	JJ.block<9,1>(0,3) = dr_x_flattened;
	JJ.block<9,1>(0,4) = dr_y_flattened;
	JJ.block<9,1>(0,5) = dr_z_flattened;
	JJ.block<3,3>(9,0) = Ri.transpose();
	JJ.block<3,3>(9,3) = -Ri.transpose() * skew(tj);

	JI = -JJ;

	Matrix4f h_x = Matrix4f::Identity();
	h_x.block<3,3>(0,0) = Ri.transpose() * Rj;
	h_x.block<3,1>(0,3) = Ri.transpose() * t_ij;

	//! Compose e
	Matrix4f temp_e = h_x - z;
	error.block<3,1>(0,0) = temp_e.block<3,1>(0,0);
	error.block<3,1>(3,0) = temp_e.block<3,1>(0,1);
	error.block<3,1>(6,0) = temp_e.block<3,1>(0,2);
	error.block<3,1>(9,0) = temp_e.block<3,1>(0,3);
}


//! --------------------- TOTAL ITERATIONS ---------------------//
void Solver::doIterations(const int iterations,
		Matrix4fVector& new_robot_poses,
		Vector3fVector& new_landmark_points,
		Stats& final_stats){
	if(_problem_dim == -1){
		cerr << "Solver init required to define params\nExit" << endl;
		return;
	}

	MatrixXf H(_problem_dim, _problem_dim);
	VectorXf b(_problem_dim);

	MatrixXf D(_problem_dim, _problem_dim);
	VectorXf dX(_problem_dim);

	float l_chi, p_chi, r_chi;
	int l_inl, p_inl, r_inl;

	cout << "\t" << "Iterate " << iterations << " times:" << endl;
	int percentage = 0;
	float flag = false;

	for (int i = 0; i < iterations; i++) {
		H.setZero();
		b.setZero();
		dX.setZero();

		//! Landmarks
		linearizeLandmarks(H, b, l_chi, l_inl); //! OK
		final_stats.l_chis.push_back(l_chi);
		final_stats.l_inliers.push_back(l_inl);

		//! Projections
		linearizeProjections(H, b, p_chi, p_inl); //! OK after a lot of iterations
		final_stats.p_chis.push_back(p_chi);
		final_stats.p_inliers.push_back(p_inl);

		//! Odom
		linearizeOdometry(H, b, r_chi, r_inl); //! OK
		final_stats.r_chis.push_back(r_chi);
		final_stats.r_inliers.push_back(r_inl);

		D.setIdentity();
		H.noalias() += D * _lambda;
		dX = (H.colPivHouseholderQr().solve(-b));
		dX.block<6,1>(0,0).setZero();
		boxPlus(dX, _robot_poses, _land_points);

		if(percentage != i * 100 / iterations){
			percentage = (int)(i * 100 / iterations);
			if(percentage % 10 == 0)
				cout << "\t" << percentage << "% completed" << endl;
		}

	}
	new_robot_poses.clear();
	new_robot_poses.resize(_robot_poses.size());
	new_robot_poses = _robot_poses;

	new_landmark_points.clear();
	new_landmark_points.resize(_land_points.size());
	new_landmark_points = _land_points;
}

//! ---------------------- UTILITIES --------------------- //
int Solver::getPoseMatrixIndex(int curr_pose_idx){
	if(curr_pose_idx > _robot_poses.size() - 1){
		cerr << "Exceedingindex\nExit" << endl;
		return -1;
	}
	return curr_pose_idx * _x_dim;
}

int Solver::getLandMatrixIndex(int curr_land_idx){
	if(curr_land_idx > _land_points.size() - 1){
			cerr << "Exceedingindex\nExit" << endl;
			return -1;
	}
	return _robot_poses.size() * _x_dim + curr_land_idx * _l_dim;
}

void Solver::boxPlus(const Eigen::VectorXf& dX, Matrix4fVector& robot_poses, Vector3fVector& land_points){
	//! Robot poses
	static_cast<int>(_x_dim);
	static_cast<int>(_l_dim);
	for(int curr_idx = 0; curr_idx < robot_poses.size(); curr_idx++){
		int matrix_idx = getPoseMatrixIndex(curr_idx);
		Vector6f dxr = dX.block<6,1>(matrix_idx,0);
		robot_poses[curr_idx] = v2t(dxr) * robot_poses[curr_idx];
	}
	//! Landmarks
	for(int curr_idx = 0; curr_idx < land_points.size(); curr_idx++){
		int land_idx = getLandMatrixIndex(curr_idx);
		Vector3f dxl = dX.block<3,1>(land_idx,0);
		land_points[curr_idx].noalias() += dxl;
	}
	return;
}

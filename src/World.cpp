/*
 * World.cpp
 *
 *  Created on: 17/feb/2017
 *      Author: istin
 */

#include "World.h"

using namespace std;
using namespace Eigen;

World::World() {
	_num_landmarks = 10;
	_num_poses = 10;
	_num_landmark_meas = _num_poses * _num_landmarks;
	_world_size = 10;
	_perturbation_dev = 1.0f;

	_K << 150,0,320,
	  0, 150,240,
	  0, 0, 1;
}

World::~World() {
	// TODO Auto-generated destructor stub
}

void World::initWorld(float n_poses,
		float n_land,
		float world_size){
	_num_poses = n_poses;
	_num_landmarks = n_land;
	_world_size = world_size;

	initPoses();
	initLandmarks();
	generateLandmarkMeas();
	generateProjectionsMeas();
	generateOdometryMeas();

	generatePerturbatedPoses();
	generatePerturbatedLandmarks();
}

void World::getPoses(std::vector<Eigen::Matrix4f>& dest){
	dest.resize(_XR_vec.size());
	for(int i = 0; i < _XR_vec.size(); i++)
		dest[i] = _XR_vec[i];
}

void World::getLandmarks(std::vector<Eigen::Vector3f>& dest){
	dest.resize(_XL_vec.size());
	for(int i = 0; i < _XL_vec.size(); i++)
		dest[i] = _XL_vec[i];
}

void World::getZl(std::vector<Eigen::Vector3f>& dest){
	dest.resize(_Zl_vec.size());
	for(int i = 0; i < _Zl_vec.size(); i++)
		dest[i] = _Zl_vec[i];
}

void World::getZp(std::vector<Eigen::Vector2f>& dest){
	dest.resize(_Zp_vec.size());
	for(int i = 0; i < _Zp_vec.size(); i++)
		dest[i] = _Zp_vec[i];
}

void World::getZr(std::vector<Eigen::Matrix4f>& dest){
	dest.resize(_Zr_vec.size());
	for(int i = 0; i < _Zr_vec.size(); i++)
		dest[i] = _Zr_vec[i];
}

void World::initPoses(void) {
	if(_num_poses == -1){
		cerr << "World not initialized!" << endl << "EXITING" << endl;
		return;
	}

	//! First pose -> identity
	_XR_true_vec.push_back(Matrix4f::Identity());

	DiagonalMatrix<float, 6> scale;
	scale.diagonal() << (float)_world_size/2.0,
			(float)_world_size/2.0,
			(float)_world_size/2.0,
			M_PI, M_PI, M_PI;

	Vector6f xr_temp;
	for(int i = 0; i < _num_poses - 1; i++){
		xr_temp = Vector6f::Random();
		_XR_true_vec.push_back(v2t(scale * xr_temp));
	}
}

void World::initLandmarks(void){
	if(_num_landmarks == -1){
		cerr << "World not initialized!" << endl << "EXITING" << endl;
		return;
	}
	Vector3f xl_temp;
	for(int i = 0; i < _num_landmarks; i++){
		xl_temp = Vector3f::Random() * _world_size;
		_XL_true_vec.push_back(xl_temp);
	}
}

void World::generateLandmarkMeas(void){
	if(_XR_true_vec.size() == 0 || _XL_true_vec.size() == 0 ){
		cerr << "XR_true or XL_true is zero!\nExit" << endl;
		return;
	}

	Matrix4f curr_xr;
	Vector3f curr_xl, zl;
	Association curr_assoc;

	for(int pose_idx = 0; pose_idx < _num_poses; pose_idx++){
		curr_assoc.x_idx = pose_idx;
		curr_xr = _XR_true_vec[pose_idx];
		for(int land_idx = 0; land_idx < _num_landmarks; land_idx++){
			curr_assoc.h_idx = land_idx;
			curr_xl = _XL_true_vec[land_idx];
			_landmark_associations.push_back(curr_assoc);
			zl = (curr_xr.block<3,3>(0,0) * curr_xl) + curr_xr.block<3,1>(0,3);
			_Zl_vec.push_back(zl);
		}
	}
}

void World::generateProjectionsMeas(void){
	if(_XR_true_vec.size() == 0 || _XL_true_vec.size() == 0 ){
		cerr << "XR_true or XL_true is zero!\nExit" << endl;
		return;
	}

	Matrix4f curr_xr;
	Vector3f curr_xl;
	Vector2f zp;
	Association curr_assoc;

	for(int pose_idx = 0; pose_idx < _num_poses; pose_idx++){
		curr_assoc.x_idx = pose_idx;
		curr_xr = _XR_true_vec[pose_idx];
		for(int land_idx = 0; land_idx < _num_landmarks; land_idx++){
			curr_assoc.h_idx = land_idx;
			curr_xl = _XL_true_vec[land_idx];

			if(projectPoint(curr_xr, curr_xl, zp)){
				_Zp_vec.push_back(zp);
				_projection_associations.push_back(curr_assoc);
			}
		}
	}
	_num_projection_meas = _Zp_vec.size();
}

bool World::projectPoint(const Eigen::Matrix4f& xr,
		const Eigen::Vector3f& xl, Eigen::Vector2f& projected_point){
	Matrix3f KT = _K * xr.block<3,3>(0,0);
	Vector3f pp =  KT * xl + xr.block<3,1>(0,3);
	pp *= (1./pp.z());

	if(pp.x() < 0 || pp.x() > IMG_COLS ||
			pp.y() < 0 || pp.y() > IMG_ROWS){
		projected_point.setZero();
		return false;
	}
	projected_point << pp.x(), pp.y();
	return true;
}

void World::generateOdometryMeas(void){
	if(_XR_true_vec.size() == 0){
		cerr << "XR_true is zero!\nExit" << endl;
		return;
	}

	Matrix4f Xi, Xj, zr;
	Association curr_assoc;
	for(int pose_idx = 0; pose_idx < _num_poses - 1; pose_idx++){
		curr_assoc.x_idx = pose_idx;
		curr_assoc.h_idx = pose_idx + 1;
		Xi = _XR_true_vec[pose_idx];
		Xj = _XR_true_vec[pose_idx + 1];

		zr = Xi.inverse() * Xj;

		_Zr_vec.push_back(zr);
		_pose_associations.push_back(curr_assoc);
	}
	_num_odometry_meas = _Zr_vec.size();
}

void World::generatePerturbatedPoses(void){
	_XR_vec.resize(_XR_true_vec.size());
	_XR_vec[0].setIdentity();

	DiagonalMatrix<float, 6> scale;
	scale.diagonal() << (float)_perturbation_dev,
					(float)_perturbation_dev,
					(float)_perturbation_dev,
					(float)_perturbation_dev,
					(float)_perturbation_dev,
					(float)_perturbation_dev;

	Vector6f xr_temp;
	Matrix4f dXR;
	for(int i = 1; i < _XR_vec.size(); i++){
		xr_temp = Vector6f::Random();
		dXR = v2t(scale * xr_temp);
		_XR_vec[i] = dXR * _XR_true_vec[i];
	}
}

void World::generatePerturbatedLandmarks(void){
	_XL_vec.resize(_XL_true_vec.size());
	Vector3f dXL = Vector3f::Random() * _perturbation_dev;
	for(int i = 0; i < _XL_vec.size(); i++){
		_XL_vec[i] = _XL_true_vec[i] + dXL;
	}
}


void World::printAllPoses(const bool flag){
	if (flag) {
		if(_XR_true_vec.size() == 0){
			cerr << "No XR_true initialized" << endl;
			return;
		}
		cout << endl;
		for (int i = 0; i < _num_poses; i++) {
			cout << "_XR_true_vec[" << i << "]:" << endl;
			cout << _XR_true_vec[i] << endl;
		}
	} else {
		if(_XR_vec.size() == 0){
			cerr << "No XR_GUESS initialized" << endl;
			return;
		}
		cout << endl;
		for (int i = 0; i < _num_poses; i++) {
			cout << "_XR__GUESS_vec[" << i << "]:" << endl;
			cout << _XR_vec[i] << endl;
		}
	}
}

void World::printAllLandmarks(const bool flag){
	if(flag){
		if(_XL_true_vec.size() == 0){
			cerr << "No XL_true initialized" << endl;
			return;
		}
		cout << endl;
		for (int i = 0; i < _num_landmarks; i++) {
			cout << "_XL_true_vec[" << i << "]:" << endl;
			cout << _XL_true_vec[i] << endl;
		}
	} else {
		if(_XL_vec.size() == 0){
			cerr << "No XL_GUESS initialized" << endl;
			return;
		}
		cout << endl;
		for (int i = 0; i < _num_landmarks; i++) {
			cout << "_XL_guess_vec[" << i << "]:" << endl;
			cout << _XL_vec[i] << endl;
		}
	}
}

void World::printLandMeas(const int idx){
	if(idx >= _num_landmark_meas){
		cerr << "Exceeding index limit" << endl;
		return;
	}
	cout << "landmark_meas[" << idx << "]:" << endl;
	cout << _Zl_vec[idx] << endl;
	cout << "generated by: ";
	cout << "P = " << _landmark_associations[idx].x_idx << "\t"
			<< "L = " <<  _landmark_associations[idx].h_idx<< endl;
}

void World::printProjMeas(const int idx){
	if(idx >= _num_projection_meas){
		cerr << "Exceeding index limit" << endl;
		return;
	}
	cout << "projection_meas[" << idx << "]:" << endl;
	cout << _Zp_vec[idx] << endl;
	cout << "generated by: ";
	cout << "P = " << _projection_associations[idx].x_idx << "\t"
			<< "L = " <<  _projection_associations[idx].h_idx<< endl;
}

void World::printOdomMeas(const int idx){
	if(idx >= _num_odometry_meas){
		cerr << "Exceeding index limit" << endl;
		return;
	}
	cout << "odometry_meas[" << idx << "]:" << endl;
	cout << _Zr_vec[idx] << endl;
	cout << "generated by: ";
	cout << "P = " << _pose_associations[idx].x_idx << "\t"
			<< "L = " <<  _pose_associations[idx].h_idx<< endl;
}

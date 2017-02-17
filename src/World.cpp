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
}

World::~World() {
	// TODO Auto-generated destructor stub
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

void World::generateLandmarkObs(void){
	if(_XR_true_vec.size() == 0 || _XL_true_vec.size() == 0 ){
		cerr << "XR_true or XL_true is zero!\nExit" << endl;
		return;
	}

	Matrix4f curr_xr;
	Vector3f curr_xl, zl;
	Association curr_assoc;

	for(int pose_idx = 0; pose_idx < _num_poses; pose_idx++){
		curr_assoc.pose_idx = pose_idx;
		curr_xr = _XR_true_vec[pose_idx];
		for(int land_idx = 0; land_idx < _num_landmarks; land_idx++){
			curr_assoc.land_idx = land_idx;
			curr_xl = _XL_true_vec[land_idx];
			_pose_land_association_vec.push_back(curr_assoc);
			zl = (curr_xr.block<3,3>(0,0) * curr_xl) + curr_xr.block<3,1>(0,3);
			_Zl_vec.push_back(zl);
		}
	}
}



void World::printAllPoses(bool flag){
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
	}
}

void World::printAllLandmarks(bool flag){
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
	}
}

void World::printMeasurements(int idx){
	if(idx > _num_landmark_meas){
		cerr << "Exeedind index limit" << endl;
		return;
	}
	cout << "landmark_meas[" << idx << "]:" << endl;
	cout << _Zl_vec[idx] << endl;
	cout << "generated by: ";
	cout << "P = " << _pose_land_association_vec[idx].pose_idx << "\t"
			<< "L = " <<  _pose_land_association_vec[idx].land_idx << endl;
}

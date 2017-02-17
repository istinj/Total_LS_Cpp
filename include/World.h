/*
 * World.h
 *
 *  Created on: 17/feb/2017
 *      Author: istin
 */
#pragma once
#ifndef WORLD_H_
#define WORLD_H_
#include <Eigen/Core>
#include <boost/random.hpp>

#include "utilities.h"


class World {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	World();
	virtual ~World();

	//! Init TRUE Poses (random)
	void initPoses(void);
	//! Init TRUE Landmarks (random)
	void initLandmarks(void);

	//! Generate Landmark Obs, supposing that
	//! each pose observes every landmark
	void generateLandmarkObs(void);

	//! If flag == true -> prints the true poses;
	//! If flag == false -> prints the estiamted poses;
	void printAllPoses(bool flag);
	void printAllLandmarks(bool flag);
	void printMeasurements(int idx);
private:
	//! General parameters of the synthetic world
	int _num_landmarks = -1;
	int _num_poses = -1;
	int _world_size = -1;
	int _num_landmark_meas = -1;

	//! Landmarks true pos
	std::vector<Eigen::Vector3f> _XL_true_vec;
	//! Landmarks est pos
	std::vector<Eigen::Vector3f> _xl_vec;

	//! Robot true poses
	std::vector<Eigen::Matrix4f> _XR_true_vec;
	//! Robot est poses
	std::vector<Eigen::Matrix4f> _xr_vec;

	//! Meas vector
	std::vector<Eigen::Vector3f> _Zl_vec;
	//! Data association poses-landmarks
	std::vector<Association> _pose_land_association_vec;

};

#endif /* WORLD_H_ */

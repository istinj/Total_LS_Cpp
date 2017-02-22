/*
 * World.h
 *
 *  Created on: 17/feb/2017
 *      Author: istin
 */
#pragma once
#ifndef WORLD_H_
#define WORLD_H_
#define IMG_COLS 540
#define IMG_ROWS 480
#include <Eigen/Core>
#include "utilities.h"


class World {
public:
	World();
	virtual ~World();

	void initWorld(float n_poses,
			float n_land,
			float world_size);

	void getPoses(std::vector<Eigen::Matrix4f>& dest);
	void getLandmarks(std::vector<Eigen::Vector3f>& dest);
	void getZl(std::vector<Eigen::Vector3f>& dest);
	void getZp(std::vector<Eigen::Vector2f>& dest);
	void getZr(std::vector<Eigen::Matrix4f>& dest);

	inline std::vector<Eigen::Matrix4f> poses(void){return _XR_vec;}
	inline std::vector<Eigen::Vector3f> landmarks(void){return _XL_vec;}
	inline std::vector<Eigen::Vector3f> zl(void){return _Zl_vec;}
	inline std::vector<Eigen::Vector2f> zp(void){return _Zp_vec;}
	inline std::vector<Eigen::Matrix4f> zr(void){return _Zr_vec;}
	inline std::vector<Association> lAssoc(void){return _landmark_associations;}
	inline std::vector<Association> pAssoc(void){return _projection_associations;}
	inline std::vector<Association> rAssoc(void){return _pose_associations;}
	inline Eigen::Matrix3f K(void){return _K;}

	//! ONLY FOR DEBUG
	//! If flag == true -> prints the true poses;
	//! If flag == false -> prints the estiamted poses;
	void printAllPoses(const bool flag);
	void printAllLandmarks(const bool flag);
	void printLandMeas(const int idx);
	void printProjMeas(const int idx);
	void printOdomMeas(const int idx);

private:
	//! Init TRUE Poses (random)
	void initPoses(void);
	//! Init TRUE Landmarks (random)
	void initLandmarks(void);

	//! Generate Landmark Measurements, supposing that
	//! each pose observes every landmark
	void generateLandmarkMeas(void);

	//! Generate Landmark Projection Measurements,
	//! supposing that each pose observes every landmark
	void generateProjectionsMeas(void);
	bool projectPoint(const Eigen::Matrix4f& xr,
			const Eigen::Vector3f& xl,
			Eigen::Vector2f& projected_point);

	//! Generate Odometry Measurements of the robot
	//! trajectory
	void generateOdometryMeas(void);

	//! Generate (WRONG) Poses and Landmarks to be
	//! optimized
	void generatePerturbatedPoses(void);
	void generatePerturbatedLandmarks(void);


	//! General parameters of the synthetic world
	int _num_landmarks = -1;
	int _num_poses = -1;
	int _num_landmark_meas = -1;
	int _num_projection_meas = -1;
	int _num_odometry_meas = -1;
	int _world_size = -1;

	float _perturbation_dev = -1;

	Eigen::Matrix3f _K;							//! Camera matrix
	std::vector<Eigen::Vector3f> _XL_true_vec;	//! Landmarks true pos
	std::vector<Eigen::Vector3f> _XL_vec;		//! Landmarks est pos

	std::vector<Eigen::Matrix4f> _XR_true_vec;	//! Robot true poses
	std::vector<Eigen::Matrix4f> _XR_vec;		//! Robot est poses

	std::vector<Eigen::Vector3f> _Zl_vec;		//! Landmark Measurements
	std::vector<Eigen::Vector2f> _Zp_vec;		//! Projection Measurements
	std::vector<Eigen::Matrix4f> _Zr_vec;		//! Odometry Measurements
	std::vector<Association> _landmark_associations;
	std::vector<Association> _projection_associations;
	std::vector<Association> _pose_associations;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

#endif /* WORLD_H_ */

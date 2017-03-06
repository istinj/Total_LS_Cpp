/*
 * Edge.h
 *
 *  Created on: 06/mar/2017
 *      Author: istin
 */

#ifndef EDGE_H_
#define EDGE_H_

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

typedef Eigen::Vector3f EdgePosePoint;
typedef Eigen::Matrix3f OmegaPosePoint;
typedef Eigen::Isometry3f EdgeOdom;
typedef Eigen::Matrix<float, 6, 6> OmegaOdom;

template <class _DataType, class _InfoMatrixType>
class Edge {
public:
	Edge();
	~Edge();

	void setEdge(const std::pair<int, int> association_,
			const _DataType data_,
			const _InfoMatrixType omega_);

	void setEdge(const std::pair<int, int> association_,
			const int sensor_ID_,
			const _DataType data_,
			const _InfoMatrixType omega_);

	inline const std::pair<int, int> getAssociation(void){return _IDassociation;};
	inline const _DataType data(void){return _data;};
	inline const _InfoMatrixType omega(void){return _Omega;};

private:
	std::pair<int, int> _IDassociation;
	int _sensor_id;

	_DataType _data;
	_InfoMatrixType _Omega;

};

template <class _DataType, class _InfoMatrixType>
Edge<_DataType, _InfoMatrixType>::Edge(){
	_IDassociation = std::pair<int, int>(-1, -1);
	_sensor_id = -1; //! also for odometry meas
}

template <class _DataType, class _InfoMatrixType>
Edge<_DataType, _InfoMatrixType>::~Edge(){
}

template <class _DataType, class _InfoMatrixType>
void Edge<_DataType, _InfoMatrixType>::setEdge(const std::pair<int, int> association_,
		const _DataType data_,
		const _InfoMatrixType omega_){
	_IDassociation = association_;
	_data = data_;
	_Omega = omega_;
}

template <class _DataType, class _InfoMatrixType>
void Edge<_DataType, _InfoMatrixType>::setEdge(const std::pair<int, int> association_,
		const int sensor_ID_,
		const _DataType data_,
		const _InfoMatrixType omega_){
	_IDassociation = association_;
	_sensor_id = sensor_ID_;
	_data = data_;
	_Omega = omega_;
}

#endif /* EDGE_H_ */

/*
 * Vertex.h
 *
 *  Created on: 06/mar/2017
 *      Author: istin
 */

#ifndef VERTEX_H_
#define VERTEX_H_

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace optimizer {

typedef Eigen::Isometry3f RobotPose;
typedef Eigen::Vector3f LandmarkXYZ;

template <class _DataType>
class Vertex{
public:
	Vertex();
	Vertex(const int id_, const _DataType& data_);
	~Vertex();

	void setVertex(const int id_, const _DataType& data_);

	inline const _DataType data(void){return _data;};
	inline const int id(void){return _id;};

private:
	_DataType _data;
	int _id;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

template <class _DataType>
Vertex<_DataType>::Vertex(){
	_id = -1;
}

template <class _DataType>
Vertex<_DataType>::Vertex(const int id_, const _DataType& data_){
	_id = id_;
	_data = data_;
}


template <class _DataType>
Vertex<_DataType>::~Vertex(){
}


template <class _DataType>
void Vertex<_DataType>::setVertex(const int id_, const _DataType& data_){
	_id = id_;
	_data = data_;
}

}/* namespace optimizer */

#endif /* VERTEX_H_ */

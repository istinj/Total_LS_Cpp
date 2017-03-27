/*
 * Hessian.h
 *
 *  Created on: 12/mar/2017
 *      Author: istin
 */

#ifndef HESSIAN_H_
#define HESSIAN_H_
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include "utilities.h"

namespace optimizer {
//! ----------------------------------------------- !//
//! ----------- Base class for Hessian ------------ !//
//! ----------------------------------------------- !//
class GenericHessian{
public:
	GenericHessian();
	virtual ~GenericHessian();

	virtual bool operator==(const GenericHessian& other_);
	virtual bool operator<(const GenericHessian& other_);
	virtual GenericHessian& operator=(const GenericHessian& other_);

	inline const std::pair<int, int>& getIndices(void) const {return _indices;};

	virtual void set(const std::pair<int, int>& indices_);
	virtual void print(void);
protected:
	std::pair<int, int> _indices;
};


//! ----------------------------------------------- !//
//! --------- Derived class for Hessian ----------- !//
//! ----------------------------------------------- !//
template<class _DataType>
class Hessian : public GenericHessian{
public:
	Hessian();
	Hessian(const std::pair<int, int>& indices_,
			const _DataType& data_);
	Hessian(const std::pair<int, int>& indices_,
			const _DataType& data_,
			const std::pair<int, int>& size_);
	virtual ~Hessian();

	virtual bool operator==(const Hessian<_DataType>& other_);
	virtual Hessian<_DataType>& operator=(const Hessian<_DataType>& other_);

	inline const std::pair<int, int>& getIndices(void) const {return _indices;};
	inline const _DataType& getData(void) const {return _data;};
	inline const int getRows(void) const {return _rows;};
	inline const int getCols(void) const {return _cols;};

	virtual void set(const std::pair<int, int>& indices_,
			const _DataType& data_);
	virtual void print(void);
protected:
	_DataType _data;
	int _rows = -1;
	int _cols = -1;
};

//! ----------------------------------------------- !//
//! --------- Derived class for Hessian ----------- !//
//! ----------------------------------------------- !//
template<class _DataType>
Hessian<_DataType>::Hessian(){
	_indices = std::make_pair(-1,-1);
}

template<class _DataType>
Hessian<_DataType>::~Hessian(){
	//! placeholder
}

template<class _DataType>
Hessian<_DataType>::Hessian(const std::pair<int, int>& indices_,
		const _DataType& data_){
	_indices = indices_;
	_data = data_;
}

template<class _DataType>
Hessian<_DataType>::Hessian(const std::pair<int, int>& indices_,
		const _DataType& data_,
		const std::pair<int, int>& size_){
	_indices = indices_;
	_data = data_;
	_rows = size_.first;
	_cols = size_.second;
}

template<class _DataType>
bool Hessian<_DataType>::operator==(const Hessian& other_){
	if(_indices == other_._indices &&
			_data == other_._data)
		return true;
	else
		return false;
}

template<class _DataType>
Hessian<_DataType>& Hessian<_DataType>::operator=(const Hessian<_DataType>& other_){
	_indices = other_._indices;
	_data = other_._data;
	return *this;
}


template<class _DataType>
void Hessian<_DataType>::set(const std::pair<int, int>& indices_,
		const _DataType& data_){
	_indices = indices_;
	_data = data_;
}

template<class _DataType>
void Hessian<_DataType>::print(void){
	std::cout << "Hessian block with indices: " << _indices.first <<
			"\t" << _indices.second << std::endl;
	std::cout << _data << "\n\n";
}


} /* namespace optimizer */

#endif /* HESSIAN_H_ */

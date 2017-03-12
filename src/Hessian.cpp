/*
 * Hessian.cpp
 *
 *  Created on: 12/mar/2017
 *      Author: istin
 */

#include "Hessian.hpp"
namespace optimizer {
//! ----------------------------------------------- !//
//! ----------- Base class for Hessian ------------ !//
//! ----------------------------------------------- !//
GenericHessian::GenericHessian(){
	_indices = std::make_pair(-1, -1);
}

GenericHessian::~GenericHessian(){
	//! placeholder
}

GenericHessian& GenericHessian::operator=(const GenericHessian& other_){
	_indices = other_._indices;
	return *this;
}

bool GenericHessian::operator==(const GenericHessian& other_){
	if(_indices == other_._indices)
		return true;
	else
		return false;
}

void GenericHessian::set(const std::pair<int, int>& indices_){
	_indices = indices_;
}

void GenericHessian::print(void){
	std::cout << "Hessian block with indices: " << _indices.first <<
			"\t" << _indices.second << std::endl;
}

} // end namespace

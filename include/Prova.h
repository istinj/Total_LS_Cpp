/*
 * Prova.h
 *
 *  Created on: 06/mar/2017
 *      Author: istin
 */

#ifndef PROVA_H_
#define PROVA_H_
#include <iostream>
#include <vector>
#include <Eigen/Core>

template <class _DataType>
class Prova {
public:
	Prova();
	virtual ~Prova();

	void push(_DataType& d);

private:
	std::vector<_DataType> _data;
};

template <class _DataType>
Prova<_DataType>::Prova() {
	// TODO Auto-generated constructor stub

};

template <class _DataType>
Prova<_DataType>::~Prova() {
	// TODO Auto-generated destructor stub
};

template <class _DataType>
void Prova<_DataType>::push(_DataType& d){
	_data.push_back(d);
	std::cout << "cazzo" << std::endl;
}

#endif /* PROVA_H_ */

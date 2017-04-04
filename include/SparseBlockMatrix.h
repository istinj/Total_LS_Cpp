/*
 * SparseBlockMatrix.h
 *
 *  Created on: 28/mar/2017
 *      Author: istin
 */

#ifndef SPARSEBLOCKMATRIX_H_
#define SPARSEBLOCKMATRIX_H_

#include <iostream>
#include <vector>
#include <Eigen/Core>

#include <boost/unordered_map.hpp>

#include "utilities.h"

namespace optimizer {

template<class DenseMatrixType>
class SparseBlockMatrix {
public:
	typedef DenseMatrixType DenseBlock;

	SparseBlockMatrix(const std::vector<int>& r_block_indices,
			const std::vector<int>& c_block_indices,
			const int n_rows_, const int n_cols_);
	virtual ~SparseBlockMatrix();
protected:
	std::vector<int> _rowBlockIndices;
	std::vector<int> _colBlockIndices;
	std::vector<boost::unordered_map<int, DenseBlock*>> _blockRowsContainer;

	bool _has_memory = true;
};

} /* namespace optimizer */

#include "SparseBlockMatrix.hpp"
#endif /* SPARSEBLOCKMATRIX_H_ */

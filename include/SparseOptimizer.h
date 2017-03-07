/*
 * SparseOptimizer.h
 *
 *  Created on: 07/mar/2017
 *      Author: istin
 */

#ifndef SPARSEOPTIMIZER_H_
#define SPARSEOPTIMIZER_H_

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <utilities.h>

#include "Vertex.hpp"
#include "Edge.hpp"
#include "Graph.h"
#include "SparseSolver.h"

namespace optimizer {

class SparseOptimizer {
public:
	SparseOptimizer();
	SparseOptimizer(const std::string& filename_);
	virtual ~SparseOptimizer();

	void incrementalGraphUpdate(void);
	void optimizeGraph(const int iterations_);

private:
	const Graph* _optimizable_graph = NULL;
	const SparseSolver* _sparse_solver = NULL;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

} /* namespace optimizer */

#endif /* SPARSEOPTIMIZER_H_ */

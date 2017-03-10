/*
 * SparseOptimizer.cpp
 *
 *  Created on: 07/mar/2017
 *      Author: istin
 */

#include "SparseOptimizer.h"

using namespace std;
using namespace Eigen;

namespace optimizer {

SparseOptimizer::SparseOptimizer() {
	// TODO Auto-generated constructor stub
	_optimizable_graph = new Graph();
	_sparse_solver = new SparseSolver();
}

SparseOptimizer::SparseOptimizer(const string& filename_) {
	// TODO Auto-generated constructor stub
	_optimizable_graph = new Graph(filename_);
	_sparse_solver = new SparseSolver(_optimizable_graph->verticesSE3(),
			_optimizable_graph->verticesXYZ(),
			_optimizable_graph->edgesOdometry(),
			_optimizable_graph->edgesPosePoint(),
			0.0, 25.0);
}

SparseOptimizer::~SparseOptimizer() {
	// TODO Auto-generated destructor stub
	delete _sparse_solver;
	delete _optimizable_graph;
}


void SparseOptimizer::incrementalGraphUpdate(void){
	return;
}

void SparseOptimizer::optimizeGraph(const int iterations_){
	_sparse_solver->oneStep();
	return;
}

} /* namespace optimizer */

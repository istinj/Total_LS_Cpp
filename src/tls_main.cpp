#include <stdio.h>
#include <iostream>
#include <set>
#include <Eigen/Core>
//#include <Eigen/Geometry>

#include "World.h"
#include "Solver.h"
#include "Graph.h"
#include "SparseOptimizer.h"

using namespace std;

int main(int argc, char const *argv[])
{

	string dataset_path;
	std::vector<string> args(argc);
	if(argc < 2){
		cerr << YELLOW << "Type -l <path-to-world.g20> to load the world from file" << RESET << endl;
		return 0;
	}

	for(int i = 1; i < argc; ++i){
		args[i] = argv[i];
		if(args[i] == "-h"){
			cerr << YELLOW << "Type -l <path-to-world.g20> to load the world from file" << RESET << endl;
		}
		if(args[i] == "-l"){
			dataset_path = argv[i+1];
			cerr << BOLDWHITE << "Loading world from " << dataset_path << RESET << endl;
			i++;
		}
		else{
			cerr << YELLOW << "Type -l <path-to-world.g20> to load the world from file" << RESET << endl;
		}
	}

	optimizer::SparseOptimizer* opt  = new optimizer::SparseOptimizer(dataset_path);
	opt->optimizeGraph(1);

	delete opt;
	return 0;
}

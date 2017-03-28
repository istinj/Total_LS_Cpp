#include <stdio.h>
#include <iostream>
#include <set>
#include <Eigen/Core>
//#include <Eigen/Geometry>

#include "Graph.h"
#include "SparseSolver.h"
#include "Hessian.hpp"

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

	optimizer::Graph* graph = new optimizer::Graph(dataset_path);
	optimizer::SparseSolver* solver = new optimizer::SparseSolver(graph->verticesSE3(),
			graph->verticesXYZ(), graph->edgesOdometry(), graph->edgesPosePoint(),
			0.0, 25.0);
	solver->oneStep();

/*
	boost::unordered_map<std::pair<int, int>, Eigen::Matrix3f> data;
	cout << data.size() << endl;
	data.insert(std::make_pair(make_pair(1,1), Eigen::Matrix3f::Identity()));
	data.insert(std::make_pair(make_pair(1,2), Eigen::Matrix3f::Identity()));
	data.insert(std::make_pair(make_pair(1,3), Eigen::Matrix3f::Identity()));
	cout << data.size() << endl;
	std::pair<int, int> n_idx(2,3);
	if(data.find(n_idx) == data.end()){
		cout << "nope" << endl;
		data.insert(make_pair(n_idx,Eigen::Matrix3f::Random()));
	} else {
		cout << "celo" << endl;
	}
	if(data.find(n_idx) == data.end()){
		cout << "nope" << endl;
		data.insert(make_pair(n_idx,Eigen::Matrix3f::Random()));
	} else {
		cout << "celo" << endl;
	}
/**/

	delete solver;
	delete graph;
/**/
	return 0;
}

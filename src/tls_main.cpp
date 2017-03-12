#include <stdio.h>
#include <iostream>
#include <set>
#include <Eigen/Core>
//#include <Eigen/Geometry>

#include "World.h"
#include "Graph.h"
#include "SparseSolver.h"
#include "Vertex.hpp"
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
	 * std::vector<std::unique_ptr<templateInterface>> v;
	 * templateClass<int> *i = new templateClass<int>();    // create new object
	 * v.push_back(std::unique_ptr<templateInterface>(i));  // put it in the vector
	 */



	std::vector<optimizer::GenericHessian*> container;
	std::set<optimizer::GenericHessian*> ssset;

	for(int i = 0; i < 3; ++i){
		optimizer::Hessian<Eigen::Matrix3f>* H1 = new optimizer::Hessian<Eigen::Matrix3f>(std::make_pair(0+i,20+i), Eigen::Matrix3f::Random());
		optimizer::Hessian<Eigen::Matrix2f>* H2 = new optimizer::Hessian<Eigen::Matrix2f>(std::make_pair(50+i,70+i), Eigen::Matrix2f::Random());
		H1->print();
		H2->print();

		container.push_back(H1);
		container.push_back(H2);

		ssset.insert(H1);
		ssset.insert(H1);
		ssset.insert(H2);
	}

	cout << endl << "NOW VECTOR" << endl << endl;

	for(int idx = 0; idx < container.size(); ++idx){
			container[idx]->print();
	}

	cout << endl << "NOW SET" << endl << endl;
	for(std::set<optimizer::GenericHessian*>::iterator it = ssset.begin(); it != ssset.end(); ++it){
		(*it)->print();
	}

	delete solver;
	delete graph;
	return 0;
}

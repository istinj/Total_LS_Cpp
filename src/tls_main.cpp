#include <stdio.h>
#include <iostream>
#include <Eigen/Core>
//#include <Eigen/Geometry>

#include "World.h"
#include "Solver.h"
#include "Graph.h"

using namespace std;


Graph* graph;

World* w;
Solver* solver;
Stats total_stats;

Matrix4fVector new_robot_poses;
Vector3fVector new_land_points;

int main(int argc, char const *argv[])
{


	graph = new Graph();
	if(argc == 2){
		graph->loadFromFile(argv[1]);
		cout << graph->numLandEdges() << "\t" << graph->numOdomEdges() << endl;
	}

	return 0;



	// --------------------------------------------------------------- //
	// --------------------------- OLD STUFF ------------------------- //
	// --------------------------------------------------------------- //
	/*
	w = new World();
	solver = new Solver();

	cout << BOLDYELLOW << "Initialize the World" <<  RESET << endl;
	w->initWorld(10.0f, 20.0f, 10.0f);
	cout << BOLDGREEN << "\tDone!" <<  RESET << endl << endl;

	if(argc < 2) {
		cerr << BOLDRED << "World is not exported!"<< endl;
		cerr << BOLDRED << "If you want to export it you must indicate a path to a folder as argument" << endl;
	} else {
		cerr << BOLDYELLOW << "Exporting world to the following path:" <<  RESET << endl;
		cerr << BOLDCYAN << argv[1] <<  RESET << endl;
		w->exportWorld(argv[1]);
		cerr << BOLDGREEN << "\tDone!" <<  RESET << endl << endl;;
	}

	int iters = 30;
	cout << BOLDYELLOW << "Initialize Solver" <<  RESET << endl;
	solver->init(w->poses(),
			w->landmarks(),
			w->zl(), w->zp(), w->zr(),
			w->lAssoc(), w->pAssoc(), w->rAssoc(),
			w->K(), 0.0f);
	solver->doIterations(iters,
			new_robot_poses, new_land_points,
			total_stats);
	cout << BOLDGREEN << "End iterations" <<  RESET << endl << endl;;

	cout << BOLDWHITE << "------------------------------ STATS: ------------------------------" << endl;
	cout << BOLDWHITE << "------------------------------ T = " << BOLDRED << "0" << BOLDWHITE << " ------------------------------" << endl;
	cout << BOLDWHITE << "CHI LAND\t" << "CHI_PROJ\t" << "CHI_ODOM" << endl;
	cout << BOLDYELLOW << total_stats.l_chis[0] << "\t\t" <<
			total_stats.p_chis[0] << "\t\t" <<
			total_stats.r_chis[0] << endl;
	cout << BOLDWHITE << "INL_LAND\t" << "INL_PROJ\t" << "INL_ODOM" << endl;
	cout << BOLDCYAN << total_stats.l_inliers[0] << "\t\t" <<
			total_stats.p_inliers[0] << "\t\t" <<
			total_stats.r_inliers[0] << RESET << endl << endl;

	cout << BOLDWHITE << "------------------------------ T = " << BOLDGREEN << iters << BOLDWHITE << " -----------------------------" << endl;
	cout << BOLDWHITE << "CHI LAND\t" << "CHI_PROJ\t" << "CHI_ODOM" << endl;
	cout << BOLDYELLOW << total_stats.l_chis[iters-1] << "\t" <<
			total_stats.p_chis[iters-1] << "\t" <<
			total_stats.r_chis[iters-1] << endl;
	cout << BOLDCYAN << total_stats.l_inliers[iters-1] << "\t\t" <<
			total_stats.p_inliers[iters-1] << "\t\t" <<
			total_stats.r_inliers[iters-1] << RESET << endl;
	return 0;/**/
}

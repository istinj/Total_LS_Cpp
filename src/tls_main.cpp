#include <stdio.h>
#include <iostream>
#include <Eigen/Core>

#include "World.h"
#include "Solver.h"

using namespace std;

World* w;
Solver* solver;
Stats total_stats;

Matrix4fVector new_robot_poses;
Vector3fVector new_land_points;

int main(int argc, char const *argv[])
{
	w = new World();
	solver = new Solver();

	cout << BOLDYELLOW << "Initialize the World" <<  RESET << endl;
	w->initWorld(10.0f, 10.0f, 10.0f);
	cout << BOLDGREEN << "\tDone!" <<  RESET << endl << endl;;

	if(argc < 2) {
		cerr << BOLDRED << "World is not exported!"<< endl;
		cerr << BOLDRED << "If you want to export it you must indicate a path to a folder as argument" << endl;
	} else {
		cerr << BOLDYELLOW << "Exporting world to the following path:" <<  RESET << endl;
		cerr << BOLDCYAN << argv[1] <<  RESET << endl;
		w->exportWorld(argv[1]);
		cerr << BOLDGREEN << "\tDone!" <<  RESET << endl << endl;;
	}

	int iters = 100;
	cout << BOLDYELLOW << "Initialize Solver" <<  RESET << endl;
	solver->init(w->poses(),
			w->landmarks(),
			w->zl(), w->zp(), w->zr(),
			w->lAssoc(), w->pAssoc(), w->rAssoc(),
			w->K());
	solver->doIterations(iters,
			new_robot_poses, new_land_points,
			total_stats);
	cout << BOLDGREEN << "End iterations" <<  RESET << endl << endl;;

//	cout << BOLDYELLOW << total_stats.p_chis[0] << "\t" << total_stats.p_chis[iters-1] << endl;
//	cout << BOLDCYAN << total_stats.p_inliers[0] << "\t" << total_stats.p_inliers[iters-1] << RESET << endl;
//	cout << BOLDYELLOW << total_stats.r_chis[0] << "\t" << total_stats.r_chis[iters-1] << endl;
//	cout << BOLDYELLOW << total_stats.l_chis[0] << "\t" << total_stats.l_chis[iters-1] << endl;
//	cout << BOLDCYAN << total_stats.r_inliers[0] << "\t" << total_stats.r_inliers[iters-1] << RESET << endl;
//	cout << BOLDCYAN << total_stats.l_inliers[0] << "\t" << total_stats.l_inliers[iters-1] << RESET << endl;

	cout << BOLDWHITE << "------------------------------ STATS: ------------------------------" << endl;
	cout << BOLDWHITE << "------------------------------ T = " << BOLDRED << "0" << BOLDWHITE << " ------------------------------" << endl;
	cout << BOLDWHITE << "CHI LAND\t" << "CHI_PROJ\t" << "CHI_ODOM" << endl;
	cout << BOLDYELLOW << total_stats.l_chis[0] << "\t\t" <<
			total_stats.p_chis[0] << "\t\t" <<
			total_stats.r_chis[0] << endl;
	cout << BOLDWHITE << "INL_LAND\t" << "INL_PROJ\t" << "INL_ODOM" << endl;
	cout << BOLDCYAN << total_stats.l_inliers[0] << "\t\t" <<
			total_stats.p_inliers[0] << RESET << "\t\t" <<
			total_stats.r_inliers[0] << endl << endl;

	cout << BOLDWHITE << "------------------------------ T = " << BOLDGREEN << iters << BOLDWHITE << " -----------------------------" << endl;
	cout << BOLDWHITE << "CHI LAND\t" << "CHI_PROJ\t" << "CHI_ODOM" << endl;
	cout << BOLDYELLOW << total_stats.l_chis[iters-1] << "\t\t" <<
			total_stats.p_chis[iters-1] << "\t\t" <<
			total_stats.r_chis[iters-1] << endl;
	cout << BOLDCYAN << total_stats.l_inliers[iters-1] << "\t\t" <<
			total_stats.p_inliers[iters-1] << RESET << "\t\t" <<
			total_stats.r_inliers[iters-1] << endl;
	return 0;
}

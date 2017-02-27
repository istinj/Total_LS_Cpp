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

	w->initWorld(10.0f, 10.0f, 10.0f);

	solver->init(w->poses(),
			w->landmarks(),
			w->zl(), w->zp(), w->zr(),
			w->lAssoc(), w->pAssoc(), w->rAssoc(),
			w->K());
	int iters = 1000;
	solver->doIterations(iters,
			new_robot_poses, new_land_points,
			total_stats);

//	for(int i = 0; i < iters; i++){
//		cout << BOLDYELLOW << total_stats.l_chis[i] << "\t" <<
//				total_stats.p_chis[i] << endl;
//		cout << BOLDCYAN << total_stats.l_inliers[i] << "\t" <<
//				total_stats.p_inliers[i] << RESET << endl;
//	}

	cout << BOLDYELLOW << total_stats.l_chis[0] << "\t" <<
			total_stats.p_chis[0] << "\t" <<
			total_stats.r_chis[0] << endl;
	cout << BOLDYELLOW << total_stats.l_chis[iters-1] << "\t" <<
			total_stats.p_chis[iters-1] << "\t" <<
			total_stats.r_chis[iters-1] << endl;
	cout << BOLDCYAN << total_stats.l_inliers[0] << "\t" <<
			total_stats.p_inliers[0] << RESET << "\t" <<
			total_stats.r_inliers[0] << endl;
	cout << BOLDCYAN << total_stats.l_inliers[iters-1] << "\t" <<
			total_stats.p_inliers[iters-1] << RESET << "\t" <<
			total_stats.r_inliers[iters-1] << endl;
	return 0;
}

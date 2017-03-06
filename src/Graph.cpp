/*
 * Graph.cpp
 *
 *  Created on: 06/mar/2017
 *      Author: istin
 */

#include "Graph.h"

#define VERTEX_3F "VERTEX_TRACKXYZ"
#define VERTEX_SE3 "VERTEX_SE3:QUAT"
#define EDGE_POSE_POINT "EDGE_SE3_TRACKXYZ"
#define EDGE_ODOM "EDGE_SE3:QUAT"

using namespace std;
using namespace Eigen;



Graph::Graph() {
	// TODO Auto-generated constructor stub

}

Graph::~Graph() {
	// TODO Auto-generated destructor stub
}

void Graph::addVertexSE3(const Vertex<RobotPose>& vertex_){
	_vertices_SE3.push_back(vertex_);
}

void Graph::addVertexXYZ(const Vertex<VertexXYZ>& vertex_){
	_vertices_XYZ.push_back(vertex_);
}

void Graph::addEdgePosePoint(const Edge<EdgePosePoint, OmegaPosePoint>& edge_){
	_edges_land.push_back(edge_);
}

void Graph::addEdgeOdom(const Edge<EdgeOdom, OmegaOdom>& edge_){
	_edges_odom.push_back(edge_);
}

void Graph::loadFromFile(const char* filename){
	fstream file(filename);

	Vertex<RobotPose> vertex_se3;
	Vertex<VertexXYZ> vertex_xyz;

	string line;
	while(getline(file, line)){
		stringstream ss(line);
		string element_type;

		// read datatype, if comment skip
		ss >> element_type;
		if(element_type == "#")
			continue;

		// process different elements
		if(element_type == VERTEX_3F){
			int id = -1;
			Vector3f p;

			ss >> id;
			ss >> p.x() >> p.y() >> p.z();

			vertex_xyz.setVertex(id, p);
			addVertexXYZ(vertex_xyz);
		}

		if(element_type == VERTEX_SE3){
			int id = -1;
			Vector3f t;
			Quaternionf q;

			ss >> id;
			ss >> t.x() >> t.y() >> t.z();
			ss >> q.x() >> q.y() >> q.z() >> q.w();

			Matrix3f rot = q.toRotationMatrix();

			RobotPose T;
			T.linear() = rot;
			T.translation() = t;

			vertex_se3.setVertex(id, T);
			addVertexSE3(vertex_se3);
		}

		if(element_type == EDGE_POSE_POINT){
			std::pair<int, int> IDs(-1,-1);
			int sens_id = -1;
			ss >> IDs.first >> IDs.second;
			ss >> sens_id;

			EdgePosePoint z_land;
			ss >> z_land.x() >> z_land.y() >> z_land.z();

			OmegaPosePoint omega_land;
			ss >> omega_land.row(0)(0) >> omega_land.row(0)(1) >> omega_land.row(0)(2) >>
					omega_land.row(1)(1) >> omega_land.row(1)(2) >>
					omega_land.row(2)(2);
			for(int i = 0; i < omega_land.rows(); ++i)
				for(int j = i + 1; j < omega_land.cols(); ++j)
					omega_land(j,i) = omega_land(i,j);

			Edge<EdgePosePoint, OmegaPosePoint> edge_pose_point;
			edge_pose_point.setEdge(IDs, sens_id, z_land, omega_land);
			addEdgePosePoint(edge_pose_point);
		}

		if(element_type == EDGE_ODOM){
			std::pair<int, int> IDs(-1,-1);
			int sens_id = -1;
			ss >> IDs.first >> IDs.second;

			Vector3f t;
			Quaternionf q;
			ss >> t.x() >> t.y() >> t.z();
			ss >> q.x() >> q.y() >> q.z() >> q.w();

			Matrix3f rot = q.toRotationMatrix();

			EdgeOdom odom_meas;
			odom_meas.linear() = rot;
			odom_meas.translation() = t;

			OmegaOdom omega_odom;
			ss >> omega_odom.row(0)(0) >> omega_odom.row(0)(1) >> omega_odom.row(0)(2) >> omega_odom.row(0)(3) >> omega_odom.row(0)(4) >> omega_odom.row(0)(5) >>
					omega_odom.row(1)(1) >> omega_odom.row(1)(2) >> omega_odom.row(1)(3) >> omega_odom.row(1)(4) >> omega_odom.row(1)(5) >>
					omega_odom.row(2)(2) >> omega_odom.row(2)(3) >> omega_odom.row(2)(4) >> omega_odom.row(2)(5) >>
					omega_odom.row(3)(3) >> omega_odom.row(3)(4) >> omega_odom.row(3)(5) >>
					omega_odom.row(4)(4) >> omega_odom.row(4)(5) >>
					omega_odom.row(5)(5);
			for(int i = 0; i < omega_odom.rows(); ++i)
				for(int j = i + 1; j < omega_odom.cols(); ++j)
					omega_odom(j,i) = omega_odom(i,j);

			Edge<EdgeOdom, OmegaOdom> edge_odom;
			edge_odom.setEdge(IDs, odom_meas, omega_odom);
			addEdgeOdom(edge_odom);
		}
	}
}

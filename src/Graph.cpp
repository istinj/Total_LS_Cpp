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

namespace optimizer {

Graph::Graph() {
	// TODO Auto-generated constructor stub

}

Graph::Graph(const Graph& graph_){
	_vertices_SE3 = graph_._vertices_SE3;
	_vertices_XYZ = graph_._vertices_XYZ;
	_edges_pose_point = graph_._edges_pose_point;
	_edges_pose_pose = graph_._edges_pose_pose;
}

Graph::Graph(const string& path_to_graph_){
	loadFromFile(path_to_graph_);
}

Graph::~Graph() {
	// TODO Auto-generated destructor stub
}

void Graph::addVertexSE3(const VertexSE3& vertex_){
	_vertices_SE3.push_back(vertex_);
}

void Graph::addVertexXYZ(const VertexXYZ& vertex_){
	_vertices_XYZ.push_back(vertex_);
}

void Graph::addEdgePosePoint(const EdgePosePoint& edge_){
	_edges_pose_point.push_back(edge_);
}

void Graph::addEdgeOdom(const EdgePosePose& edge_){
	_edges_pose_pose.push_back(edge_);
}

void Graph::loadFromFile(const string& filename){
	cout << BOLDYELLOW << "\t" << "Opening file " << filename << RESET << endl;
	fstream file(filename);

	int p_idx = 0;
	int l_idx = 0;
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
			LandmarkXYZ p;

			ss >> id;
			ss >> p.x() >> p.y() >> p.z();

			VertexXYZ vertex_xyz;
			vertex_xyz.setVertex(id, p, l_idx);
			addVertexXYZ(vertex_xyz);
			l_idx++;
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

			VertexSE3 vertex_se3;
			vertex_se3.setVertex(id, T, p_idx);
			addVertexSE3(vertex_se3);
			p_idx++;
		}

		if(element_type == EDGE_POSE_POINT){
			std::pair<int, int> IDs(-1,-1);
			int sens_id = -1;
			ss >> IDs.first >> IDs.second;
			ss >> sens_id;

			PointMeas z_land;
			ss >> z_land.x() >> z_land.y() >> z_land.z();

			OmegaPoint omega_land;
			ss >> omega_land.row(0)(0) >> omega_land.row(0)(1) >> omega_land.row(0)(2) >>
					omega_land.row(1)(1) >> omega_land.row(1)(2) >>
					omega_land.row(2)(2);
			for(int i = 0; i < omega_land.rows(); ++i)
				for(int j = i + 1; j < omega_land.cols(); ++j)
					omega_land(j,i) = omega_land(i,j);

			EdgePosePoint edge_pose_point;
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

			PoseMeas odom_meas;
			odom_meas.linear() = rot;
			odom_meas.translation() = t;

			OmegaPose omega_odom;
			ss >> 	omega_odom.row(0)(0) >> omega_odom.row(0)(1) >> omega_odom.row(0)(2) >> omega_odom.row(0)(3) >> omega_odom.row(0)(4) >> omega_odom.row(0)(5) >>
					omega_odom.row(1)(1) >> omega_odom.row(1)(2) >> omega_odom.row(1)(3) >> omega_odom.row(1)(4) >> omega_odom.row(1)(5) >>
					omega_odom.row(2)(2) >> omega_odom.row(2)(3) >> omega_odom.row(2)(4) >> omega_odom.row(2)(5) >>
					omega_odom.row(3)(3) >> omega_odom.row(3)(4) >> omega_odom.row(3)(5) >>
					omega_odom.row(4)(4) >> omega_odom.row(4)(5) >>
					omega_odom.row(5)(5);
			for(int i = 0; i < omega_odom.rows(); ++i)
				for(int j = i + 1; j < omega_odom.cols(); ++j)
					omega_odom(j,i) = omega_odom(i,j);
			EdgePosePose edge_odom;
			edge_odom.setEdge(IDs, odom_meas, omega_odom);
			addEdgeOdom(edge_odom);
		}
	}
	cout << BOLDGREEN << "\t" << "File loaded successfully:" << endl;
	cout << "\t" << _vertices_SE3.size() << " Vertices SE3\n\t" << _vertices_XYZ.size() << " Vertices XYZ" << endl;
	cout << "\t" <<	_edges_pose_point.size() << " Edges XYZ\n\t" << _edges_pose_pose.size() << " Edges Odometry" << RESET << endl;
}
}/* namespace optimizer */

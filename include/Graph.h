/*
 * Graph.h
 *
 *  Created on: 06/mar/2017
 *      Author: istin
 */

#ifndef GRAPH_H_
#define GRAPH_H_

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "utilities.h"
#include "Vertex.hpp"
#include "Edge.hpp"

class Graph {
public:
	Graph();
	virtual ~Graph();

	//! TODO: load from file
	//! TODO: add variable
	//! TODO: add factor (different kinds)
	void loadFromFile(const char* filename);
	void addVertexSE3(const Vertex<RobotPose>& vertex);
	void addVertexXYZ(const Vertex<VertexXYZ>& vertex);
	void addEdgePosePoint(const Edge<EdgePosePoint, OmegaPosePoint>& edge_);
	void addEdgeOdom(const Edge<EdgeOdom, OmegaOdom>& edge_);

	inline const int numSE3Vertices(void){return _vertices_SE3.size();};
	inline const int numXYZVertices(void){return _vertices_XYZ.size();};
	inline const int graphSize(void){return _vertices_SE3.size() * _vertices_XYZ.size();};

	inline const int numOdomEdges(void){return _edges_odom.size();};
	inline const int numLandEdges(void){return _edges_land.size();};

private:
	//! TODO: List of vertices
	//! TODO: List of edges
	std::vector<Vertex<RobotPose>> _vertices_SE3;
	std::vector<Vertex<VertexXYZ>> _vertices_XYZ;

	std::vector<Edge<EdgePosePoint, OmegaPosePoint>> _edges_land;
	std::vector<Edge<EdgeOdom, OmegaOdom>> _edges_odom;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

#endif /* GRAPH_H_ */

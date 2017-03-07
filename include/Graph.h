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

namespace optimizer {

	typedef Vertex<RobotPose> VertexSE3;
	typedef Vertex<LandmarkXYZ> VertexXYZ;
	typedef Edge<LandmarkMeas, OmegaLandmark> EdgePosePoint;
	typedef Edge<OdometryMeas, OmegaOdom> EdgeOdometry;


	class Graph {
	public:
		Graph();
		Graph(const Graph& graph);
		Graph(const std::string& path_to_graph_);
		virtual ~Graph();

		//! TODO: load from file
		//! TODO: add variable
		//! TODO: add factor (different kinds)
		void loadFromFile(const std::string& filename);
		void addVertexSE3(const VertexSE3& vertex);
		void addVertexXYZ(const VertexXYZ& vertex);
		void addEdgePosePoint(const EdgePosePoint& edge_);
		void addEdgeOdom(const EdgeOdometry& edge_);

		inline const std::vector<VertexSE3>& verticesSE3(void) const {return _vertices_SE3;};
		inline const std::vector<VertexXYZ>& verticesXYZ(void) const {return _vertices_XYZ;};
		inline const std::vector<EdgeOdometry>& edgesOdometry(void) const {return _edges_odom;};
		inline const std::vector<EdgePosePoint>& edgesPosePoint(void) const {return _edges_land;};

		inline const int numSE3Vertices(void) const {return _vertices_SE3.size();};
		inline const int numXYZVertices(void) const {return _vertices_XYZ.size();};
		inline const int graphSize(void) const {return _vertices_SE3.size() * _vertices_XYZ.size();};
		inline const int numOdomEdges(void) const {return _edges_odom.size();};
		inline const int numLandEdges(void) const {return _edges_land.size();};

	private:
		//! TODO: List of vertices
		//! TODO: List of edges
		std::vector<VertexSE3> _vertices_SE3;
		std::vector<VertexXYZ> _vertices_XYZ;

		std::vector<EdgePosePoint> _edges_land;
		std::vector<EdgeOdometry> _edges_odom;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		friend class SparseOptimizer;
	};
}
#endif /* GRAPH_H_ */
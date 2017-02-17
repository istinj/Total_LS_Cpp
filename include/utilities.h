#pragma once
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define PRINT_VAR(x) std::cout << #x << std::endl << x << std::endl;

typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;

struct Association{
	int pose_idx;
	int land_idx;
};

Eigen::Matrix4f v2t(const Vector6f& v);


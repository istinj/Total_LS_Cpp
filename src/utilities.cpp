#include "utilities.h"
using namespace Eigen;

Matrix4f v2t(const Vector6f& v){
	Matrix4f T = Matrix4f::Identity();
	Matrix3f Rx, Ry, Rz;
	Rx = AngleAxisf(v(3), Vector3f::UnitX());
	Ry = AngleAxisf(v(4), Vector3f::UnitY());
	Rz = AngleAxisf(v(5), Vector3f::UnitZ());
	T.block<3,3>(0,0) = Rx * Ry * Rz;
	T.block<3,1>(0,3) = v.block<3,1>(0,0);
	return T;
}

Eigen::Matrix3f skew(const Eigen::Vector3f& p)
{
	Eigen::Matrix3f s;
	s <<
		0,  -p.z(), p.y(),
		p.z(), 0,  -p.x(),
		-p.y(), p.x(), 0;
	return s;
}

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

Eigen::Vector3f random3f(void){
	Eigen::Vector3f v;
	boost::mt19937 rng;
	boost::uniform_01<> uniform;
	boost::variate_generator<boost::mt19937, boost::uniform_01<>> generator(rng, uniform);
	for (int i = 0; i < 3; ++i) {
		v(i) = (float)(generator() - 0.5);
	}
	return v;
}
Eigen::Matrix<float, 6, 1> random6f(void){
	Vector6f v;
	boost::mt19937 rng;
	boost::uniform_01<> uniform;
	boost::variate_generator<boost::mt19937, boost::uniform_01<>> generator(rng, uniform);
	for (int i = 0; i < 6; ++i) {
		v(i) = (float)(generator() - 0.5);
	}
	return v;
}

/*************************************** 
* 
* LanXin TECH, All Rights Reserverd. 
* Created at Thu May 16 10:27:58 2019
* Contributor: Ling Shi, Ph.D 
* Email: lshi@robvision.cn 
* 
***************************************/ 

#define _USE_MATH_DEFINES
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

namespace lanXin {

// hand eye calibration
// ---------------------------------------------

typedef Eigen::Vector3f			Geo3d;
typedef Eigen::Matrix3f			GeoMat3;
typedef Eigen::Matrix4f			GeoMat4;
typedef GeoMat3					RotMat;
typedef Eigen::Isometry3f		GeoTransform;

#define XEPS					1e-6

template<typename T>
inline T lxmin(const T& a, const T& b)
{
	return a < b ? a : b;
}

template<typename T>
inline T lxmax(const T& a, const T& b)
{
	return a > b ? a : b;
}

template<typename T>
inline T lxsq(const T& val)
{
	return val*val;
}

template<typename T>
inline T lxabs(const T& val)
{
	return std::abs(val);
}

template<typename T>
inline T non_zero(const T& val)
{
	return lxabs(val) > 1e-6;
}


// Rodrigues transformation
Geo3d rodrigues2(const RotMat& matrix);


inline GeoMat3 fromEulers(float rx, float ry = .0f, float rz = .0f)
{
	Eigen::AngleAxisf quat = Eigen::AngleAxisf(rx, Geo3d::UnitX());
	if (non_zero(ry))
	{
		quat =  Eigen::AngleAxisf(ry, Geo3d::UnitY()) * quat;
	}
	if(non_zero(rz))
		quat = Eigen::AngleAxisf(rz, Geo3d::UnitZ()) * quat;
	return quat.matrix();
}

inline GeoTransform getTransM(Geo3d t, Geo3d eulers)
{
	GeoTransform H;
	H.setIdentity();
	H.linear() = fromEulers(eulers[0], eulers[1], eulers[2]);
	H.translation() = t;
	return H;
}


// Solve AX = XB Problem
// cv::Mat calibrateHandEye(std::vector<cv::Mat> Hgij, std::vector<cv::Mat> Hcij);

enum HandEyeType
{
	EyeToHand,
	EyeInHand
};

// calibrate Hand to Eye
//@ vH_robot: robot pose (read from the robot)
//@ vH_mark: mark pose in camera (computed from the camera)
GeoTransform calibrateHandEye(std::vector<GeoTransform>& vH_robot, std::vector<GeoTransform>& vH_mark, HandEyeType t = EyeToHand);

GeoTransform sovleAXequalXB(std::vector<GeoTransform>& vA, std::vector<GeoTransform>& vB);

} /* End of namespace lanXin */ 


/*************************************** 
* 
* LanXin TECH, All Rights Reserverd. 
* Created at Thu May 16 10:27:58 2019
* Contributor: Ling Shi, Ph.D 
* Email: lshi@robvision.cn 
* 
***************************************/ 

#include "common.h"

#include <Eigen/SVD>
#include <iostream>
#include <fstream>

using namespace std;

namespace lanXin {


RotMat skew(Geo3d v) 
{
	RotMat rot;
	rot.setZero();

	rot(0, 1) = -v(2);
	rot(0, 2) = v(1);
	rot(1, 2) = -v(0);

	rot(1, 0) = -rot(0, 1);
	rot(2, 0) = -rot(0, 2);
	rot(2, 1) = -rot(1, 2);

	return rot;
}

Geo3d rodrigues2(const RotMat& matrix)
{
	Eigen::JacobiSVD<Eigen::Matrix3f> svd(matrix, Eigen::ComputeFullV | Eigen::ComputeFullU);
	RotMat R = svd.matrixU() * svd.matrixV().transpose();

	double rx = R(2, 1) - R(1, 2);
	double ry = R(0, 2) - R(2, 0);
	double rz = R(1, 0) - R(0, 1);

	double s = sqrt((rx*rx + ry*ry + rz*rz)*0.25);
	double c = (R.trace() - 1) * 0.5;
	c = c > 1. ? 1. : c < -1. ? -1. : c;

	double theta = acos(c);

	if (s < XEPS)
	{
		double t;

		if (c > 0)
			rx = ry = rz = 0;
		else
		{
			t = (R(0, 0) + 1)*0.5;
			rx = sqrt(std::max(t, 0.0));
			t = (R(1, 1) + 1)*0.5;
			ry = sqrt(std::max(t, 0.0)) * (R(0, 1) < 0 ? -1.0 : 1.0);
			t = (R(2, 2) + 1)*0.5;
			rz = sqrt(std::max(t, 0.0)) * (R(0, 2) < 0 ? -1.0 : 1.0);

			if (fabs(rx) < fabs(ry) && fabs(rx) < fabs(rz) && (R(1, 2) > 0) != (ry*rz > 0))
				rz = -rz;
			theta /= sqrt(rx*rx + ry*ry + rz*rz);
			rx *= theta;
			ry *= theta;
			rz *= theta;
		}
	}
	else
	{
		double vth = 1 / (2 * s);
		vth *= theta;
		rx *= vth; ry *= vth; rz *= vth;
	}
	return Eigen::Vector3d(rx, ry, rz).cast<float>();
}

GeoTransform calibrateHandEye(std::vector<GeoTransform>& vH_robot, std::vector<GeoTransform>& vH_mark, HandEyeType t)
{
	//Eigen::Matrix4f rt;
	const int n = lxmin(vH_robot.size(), vH_robot.size());
	if(n <3)
	{
		printf("At lease 3 point-pairs.\n");
		return GeoTransform();
	}

	printf("Start to calibrate with %d point-pairs.\n", n);
  
	std::vector<GeoTransform> vA, vB;

	for (int i = 0; i < n; i++)
	{
		for (int j = i + 1; j < n; ++j)
		{
			//if(i == 0 && j==i+1) continue;
			if (t == EyeToHand)
			{
				GeoTransform A = vH_robot[j] * vH_robot[i].inverse();
				GeoTransform B = vH_mark[j] * vH_mark[i].inverse();			 

				vA.push_back(A);
				vB.push_back(B);
			}
			else if (t == EyeInHand)
			{

				GeoTransform A = vH_robot[j].inverse() * vH_robot[i];
				GeoTransform B = vH_mark[j] * vH_mark[i].inverse();

				vA.push_back(A);
				vB.push_back(B);
			}
		}
	}
	

	GeoTransform H = sovleAXequalXB(vA, vB);

 

	return H;
}


Eigen::MatrixXf svdInverse(Eigen::MatrixXf  A)
{
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);//M=USV*
	float  pinvtoler = 1.e-6; //tolerance
	int row = A.rows();
	int col = A.cols();
	int k = min(row, col);
	Eigen::MatrixXf X = Eigen::MatrixXf::Zero(col, row);
	Eigen::MatrixXf singularValues_inv = svd.singularValues();//奇异值
	Eigen::MatrixXf singularValues_inv_mat = Eigen::MatrixXf::Zero(col, row);
	for (long i = 0; i < k; ++i) {
		if (singularValues_inv(i) > pinvtoler)
			singularValues_inv(i) = 1.0 / singularValues_inv(i);
		else singularValues_inv(i) = 0;
	}
	for (long i = 0; i < k; ++i)
	{
		singularValues_inv_mat(i, i) = singularValues_inv(i);
	}
	X = (svd.matrixV())*(singularValues_inv_mat)*(svd.matrixU().transpose());//X=VS+U*

	return X;
}

GeoTransform sovleAXequalXB(std::vector<GeoTransform>& vA, std::vector<GeoTransform>& vB)
{
	GeoTransform H;
	H.setIdentity();
	if (vA.size() != vB.size())
	{
		printf("A and B must be same size.\n");
		return H;
	}

	const int n = vA.size();

	RotMat R_a, R_b;
	Geo3d r_a, r_b;

	Eigen::MatrixXf A(n*3, 3);
	Eigen::MatrixXf b(n*3, 1);
	A.setZero();
	b.setZero(); 

	for (int i = 0; i < n; ++i)
	{
		R_a = vA[i].linear();
		R_b = vB[i].linear();
		
		Geo3d rod_a = rodrigues2(R_a);
		Geo3d rod_b = rodrigues2(R_b);

		float theta_a = rod_a.norm();
		float theta_b = rod_b.norm();

		rod_a /= theta_a;
		rod_b /= theta_b;

		Geo3d P_a = 2*sin(theta_a/2)*rod_a;
		Geo3d P_b = 2*sin(theta_b/2)*rod_b;		 

		Eigen::Matrix3f rot = skew(Geo3d(P_b+P_a));
		Geo3d v = P_b - P_a;
		//for (int row = 3 * i; row < 3 * i + 3; ++row)
		//{
		//	for (int col = 0; col < 3; ++col)
		//	{
		//		A(row, col) = rot(row - 3 * i, col);
		//	}

		//	b(row) = v(row - 3 * i);
		//}

		//cout << A.middleRows(3 * i, 3 * i + 3) << endl;
		//cout << rot << endl;
		A.middleRows(3 * i, 3) = rot;
		b.middleRows(3*i, 3) = v;
	}
	//Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

	// 3 by 3*n 
	Eigen::MatrixXf pinA = svdInverse(A); 

	// 3 by 1 = 3 by 3*n multi 3*n by 1
	Geo3d H_ba_prime = pinA * b;
	 
	Geo3d H_ba = 2 * H_ba_prime / sqrt(1 + lxsq(H_ba_prime.norm()));

	// 1 by 3
	Eigen::MatrixXf H_ba_Trs = H_ba.transpose();

	RotMat R_ba = (1 - lxsq(H_ba.norm()) / 2) * RotMat::Identity() 
		+ 0.5 * (H_ba * H_ba_Trs + sqrt(4 - lxsq(H_ba.norm()))*skew(H_ba));
	

	A.setZero();
	b.setZero();
	for (int i = 0; i < n; ++i)
	{
		RotMat AA = vA[i].linear() - RotMat::Identity();
		Geo3d bb = R_ba * vB[i].translation() - vA[i].translation();
		//for (int row = 3 * i; row < 3 * i + 3; ++row)
		//{
		//	for (int col = 0; col < 3; ++col)
		//	{
		//		A(row, col) = AA(row - 3 * i, col);
		//	}

		//	b(row) = bb(row - 3 * i);
		//}
		A.middleRows(3 * i, 3) = AA;
		b.middleRows(3 * i, 3) = bb;
	}
	pinA = svdInverse(A);
	Geo3d t_ba = pinA * b;
	H.linear() = R_ba;
	H.translation() = t_ba;

	// check
	for(int i = 0; i<n; ++i)
	{
		GeoTransform AX = vA[i] * H;
		GeoTransform XB = H * vB[i];


		Geo3d angles1 = AX.linear().eulerAngles(0, 1, 2);
		Geo3d angles2 = XB.linear().eulerAngles(0, 1, 2);
		// cout << i << " Dist Error: " << (AX.translation() - XB.translation()).norm()
		// 	<< ". Rotation Error: " << (angles1 - angles2).transpose() << endl;
	}
	
	std::string fname("H.txt");	
	printf("Calibration Finished. Saved at %s.\n", fname.c_str());

	cout << "H: \n" << H.matrix() << endl;
	ofstream out(fname);
	out <<"# Calibrated At " << fname  << endl;
	out << H.matrix() << endl;
	out.close(); 

	return H;
}






} /* End of namespace lanXin */ 


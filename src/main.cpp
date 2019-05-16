/*************************************** 
* 
* LanXin TECH, All Rights Reserverd. 
* Created at Thu May 16 10:27:58 2019
* Contributor: Ling Shi, Ph.D 
* Email: lshi@robvision.cn 
* 
***************************************/ 

#include "common.h"
#include <iostream>

using namespace lanXin;
using namespace std;



int main(int argc, char *argv[])
{	

	Geo3d O_world(0, 0, 0);

	// camera in world, unknown, to calibrate
	// transform camera to world, to estimate
	GeoTransform H_c_in_w = getTransM(Geo3d(0.8, 0.12, 2.4), Geo3d(M_PI*1.02, -0.12*M_PI, M_PI *0.51)); 

	cout << "camera in world:\n" << H_c_in_w.matrix() << endl;

	// grid in end, unkonwn, but we don't not have to solve it
	GeoTransform H_g_in_e = getTransM(Geo3d(0.10, 0.20, -0.401), Geo3d(0.20, -0.20, 0.20));

	// Grid in world pose list
	vector<GeoTransform> vH_e_in_w, vH_g_in_w; 
	vH_e_in_w.push_back(getTransM(Geo3d(0.56, 0.4, 0.3), Geo3d(0.2, 0.2, 0)));
	vH_e_in_w.push_back(getTransM(Geo3d(0.6, -0.14, 0.3), Geo3d(-0.3, -0.17, 0)));
	vH_e_in_w.push_back(getTransM(Geo3d(0.98, 0.4, 0.40), Geo3d(0.2, 0.15, 0.1)));
	vH_e_in_w.push_back(getTransM(Geo3d(0.85, -0.2, 0.32), Geo3d(-0.19, 0.2, 0))); 

	int n = vH_e_in_w.size();
	for(int i=0; i<n; ++i)
	{
		vH_g_in_w.push_back(vH_e_in_w[i] * H_g_in_e);
	} 

	// grid in camera = H_c_in_w^(-1) * H_g_in_w;
	vector<GeoTransform> vH_g_in_c;
	for (auto it = vH_g_in_w.begin(); it!= vH_g_in_w.end(); ++it)
	{
		vH_g_in_c.push_back(H_c_in_w.inverse() * (*it));
	}

	// add random noise to the input data
	for(int i=0; i<n; ++i)
	{
		// cout << rng.uniform(-0.01, 0.01) << endl;
		
		// Geo3d v = Geo3d(rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01));

		// vH_g_in_c[i].translation() += v/102;
		
		//Geo3d eulers = vH_g_in_c[i].linear().eulerAngles(2, 1, 0);
		// v = Geo3d(rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01)) /100;

		//cout <<"1 "<< vH_g_in_c[i].linear() << endl;
		//cout <<"2 "<<  fromEulers(eulers[2], eulers[1], eulers[0]) << endl;
		// vH_g_in_c[i].linear() = vH_g_in_c[i].linear() * fromEulers(v[0], v[1], v[2]);
	}


	// compare
	for (int i = 0; i < vH_g_in_c.size(); ++i)
	{
		GeoTransform result = H_c_in_w * vH_g_in_c[i];
		GeoTransform g_in_w = vH_g_in_w[i];
		// cout << "Compare: --- At " << i << endl << result.matrix() - g_in_w.matrix() << endl;
	}

	// compose A and B
	GeoTransform H = calibrateHandEye(vH_e_in_w, vH_g_in_c);

	return 0;
}
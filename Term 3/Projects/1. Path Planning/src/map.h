#ifndef MAP_H
#define MAP_H

#include <fstream>
#include<iostream>
#include<string>
#include <vector>
#include <sstream>
#include "spline.h"
#include "constants.h"


using namespace std;

class Map {


	private:

		tk::spline wp_x_spline;
		tk::spline wp_y_spline;
		tk::spline wp_dx_spline;
		tk::spline wp_dy_spline;

		vector<double> maps_x;
		vector<double> maps_y;
		vector<double> maps_s;


	public:

		Map(){};
		Map(string map_file);

		~Map(){};

		vector<double> getXY(double s, double d);
		vector<double> getFrenet(double x, double y, double theta);

		int ClosestWaypoint(double x, double y);
		int NextWaypoint(double x, double y, double theta);

};









#endif
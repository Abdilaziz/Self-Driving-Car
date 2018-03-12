#ifndef MAP_H
#define MAP_H

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

#include "spline.h"
#include "constants.h"


using namespace std;

class Map {


	private:

		vector<double> map_waypoints_x;
		vector<double> map_waypoints_y;
		vector<double> map_waypoints_s;
		vector<double> map_waypoints_dx;
		vector<double> map_waypoints_dy;

		vector<double> interpolated_local_waypoints_s;
		vector<double> interpolated_local_waypoints_x;
		vector<double> interpolated_local_waypoints_y;
		vector<double> interpolated_local_waypoints_dx;
		vector<double> interpolated_local_waypoints_dy;



	public:

		Map(){};
		Map(string map_file);

		~Map(){};

		vector<double> getXY(double s, double d);
		vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y, vector<double> maps_s);

		int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);
		int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

		void update_local_waypoints(double car_x, double car_y, double car_yaw);

		vector<double> interpolate_points(vector<double> pts_x, vector<double> pts_y, double interval, int output_size);
		vector<double> interpolate_points(vector<double> pts_x, vector<double> pts_y, vector<double> eval_at_x);

		vector<double> getLocalWaypointsS(){return this->interpolated_local_waypoints_s;};
		vector<double> getLocalWaypointsX(){return this->interpolated_local_waypoints_x;};
		vector<double> getLocalWaypointsY(){return this->interpolated_local_waypoints_y;};
		vector<double> getLocalWaypointsDX(){return this->interpolated_local_waypoints_dx;};
		vector<double> getLocalWaypointsDY(){return this->interpolated_local_waypoints_dy;};

};









#endif
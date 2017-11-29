#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <math.h>
#include "constants.h"
#include "map.h"

using namespace std;

class Vehicle {

	private:

		int id;
		double x;
		double y;

		double s;
		double s_d;
		double s_dd;

		double d;
		double d_d;
		double d_dd;
		double yaw;

		LANES lane;


	public:

		Vehicle();
		Vehicle(int id, double x, double y, double v, double s, double d );
		~Vehicle(){};

		void set_main_vehicle_values(Map map, double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, 
                                          vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d);

		int getID(){return this->id;};

		double getX(){return this->x;};
		double getY(){return this->y;};

		double getS(){return this->s;};
		double getS_D(){return this->s_d;};
		double getS_DD(){return this->s_dd;};

		double getD(){return this->d;};
		double getD_D(){return this->d_d;};
		double getD_DD(){return this->d_dd;};


		double getYAW(){return this->yaw;};

		LANES getLane(){return this->lane;};

		vector<vector<double>> generatePrediction(double traj_start_time, double duration);


};





#endif
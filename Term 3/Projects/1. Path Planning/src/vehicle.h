#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include "constants.h"

using namespace std;

class Vehicle {

	private:

		int id;
		double x;
		double y;
		double v;
		double s;
		double d;
		double yaw;

		LANES lane;


		vector<double> previous_s;
		vector<double> previous_d;



	public:

		Vehicle();
		Vehicle(int id, double x, double y, double v, double s, double d);
		~Vehicle(){};

		void set_main_vehicle_values(double x, double y, double v, double s, double d, double yaw);

		int getID();
		double getX();
		double getY();
		double getV();
		double getS();
		double getD();
		double getYAW();

		LANES getLane();

		vector<double> getPREV_S();
		vector<double> getPREV_D();

		void setPREV_S(vector<double> previous_s);
		void setPREV_D(vector<double> previous_d);


};





#endif
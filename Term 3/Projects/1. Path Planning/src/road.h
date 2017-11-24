#ifndef ROAD_H
#define ROAD_H


#include <vector>
#include <math.h>
#include "constants.h"
#include "vehicle.h"

using namespace std;

class Road {

	private:
		vector<Vehicle> left_lane;
		vector<Vehicle> center_lane;
		vector<Vehicle> right_lane;
		double vehicle_ahead_speed;


	public:
		Road(){};
		~Road(){};

		void setVehiclesOnRoad(vector<Vehicle> left_lane, vector<Vehicle> center_lane, vector<Vehicle> right_lane);

		bool isVehicleAhead(Vehicle &my_car, LANES lane);
		bool isLaneChangeSafe(Vehicle &my_car, LANES lane);
		LANES availableLane(Vehicle &my_car);

		double getSPEED() {return this->vehicle_ahead_speed;};
		void setSPEED(double vehicle_ahead_speed) { this->vehicle_ahead_speed = vehicle_ahead_speed; };
		

};








#endif
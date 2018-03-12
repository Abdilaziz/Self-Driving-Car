#ifndef ROAD_H
#define ROAD_H


#include <vector>
#include <map>
#include <math.h>
#include "constants.h"
#include "vehicle.h"

using namespace std;

class Road {

	private:
		Vehicle vehicle_ahead;
		Vehicle vehicle_behind;

		map<int, vector<Vehicle>> predictions;


	public:
		Road(){};
		~Road(){};

		void setVehiclesOnRoad(vector<Vehicle> left_lane, vector<Vehicle> center_lane, vector<Vehicle> right_lane);

		bool isVehicleAhead(double s, LANES lane);

		bool isVehicleBehind(double s, LANES lane);

		Vehicle getVehicleAhead() { return this->vehicle_ahead;};
		Vehicle getVehicleBehind() { return this->vehicle_behind; };
	
		map<int, vector<Vehicle>> getPredictions(){ return this->predictions; };

		void setPredictions(map<int,vector<Vehicle>> predictions) { this->predictions = predictions; };
		
};








#endif
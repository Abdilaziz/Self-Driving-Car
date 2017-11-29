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
		vector<Vehicle> left_lane;
		vector<Vehicle> center_lane;
		vector<Vehicle> right_lane;
		Vehicle vehicle_ahead;

		map<int, vector<vector<double>>> predictions;


	public:
		Road(){};
		~Road(){};

		void setVehiclesOnRoad(vector<Vehicle> left_lane, vector<Vehicle> center_lane, vector<Vehicle> right_lane);

		bool isVehicleAhead(Vehicle &my_car, LANES lane);
		bool isLaneChangeSafe(Vehicle &my_car, LANES lane);

		bool isVehicleAhead(Vehicle &my_car, int lane);
		bool isLaneChangeSafe(Vehicle &my_car, int lane);

		LANES availableLane(Vehicle &my_car);

		Vehicle getVehicleAhead() {return this->vehicle_ahead;};
		void setVehicleAhead(Vehicle vehicle_ahead) { this->vehicle_ahead = vehicle_ahead; };


		void generatePredctions(double traj_start_time, double duration);
		
		map<int,vector<vector<double>>> getPredictions(){ return this->predictions; };

		void setPredictions(map<int,vector<vector<double>>> predictions) { this->predictions = predictions; };
};








#endif
#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;

double calculate_cost(Vehicle vehicle, map<int, vector<Vehicle>> predictions, vector<Vehicle> trajectory);

double goal_distance_cost(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions, map<string, double> data);

double inefficiency_cost(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions, map<string, double> data);

double lane_speed(map<int, vector<Vehicle>> predictions, int lane);

map<string, double> get_helper_data(Vehicle vehicle, vector<Vehicle> trajectory, map<int, vector<Vehicle>> predictions);

#endif
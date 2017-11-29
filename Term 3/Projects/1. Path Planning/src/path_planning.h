#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include <vector>
#include <string>

#include "map.h"
#include "vehicle.h"
#include "road.h"
#include "constants.h"
#include "spline.h"
#include "costs.h"

using namespace std;

class Path_Planner {

	private:

		int new_points;
		STATES state;

		Map map;
		vector<vector<double>> current_trajectory;
		Road road;
		Vehicle my_car;
		int prev_path_size;
		double duration;


		vector<double> JMT(vector< double> start, vector <double> end, double T);

		double lane_to_d(LANES lane);
		LANES d_to_lane(double d);

		vector<STATES> possible_next_states();

		vector<vector<double>> state_transition(vector<STATES> possible_next_states);
		vector<vector<double>> basic_state_transition();

		vector<vector<double>> get_target(STATES state);


		vector<vector<double>> startVehicle();
		vector<vector<double>> keep_in_lane();
		vector<vector<double>> slow_down();
		vector<vector<double>> change_lane(STATES state);
		vector<vector<double>> change_lane(LANES target_lane);

		void set_state(LANES current_lane, LANES target_lane);

		vector<vector<double>> proeduce_path(vector<double> target_s, vector<double> target_d);
		

		
		vector<vector<double>> create_JMT(vector<double> start_s, vector<double> end_s, vector<double> start_d, vector<double> end_d, double duration);



	public:

		Path_Planner();
		~Path_Planner(){};

		vector<vector<double>> get_new_trajectory(Map &map, Road &road, Vehicle &my_car, vector<vector<double>> trajectory, int prev_path_size);



};




#endif
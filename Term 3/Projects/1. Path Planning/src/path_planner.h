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
#include "helper.h"

using namespace std;

class Path_Planner {

    private:

		STATES state;
		Helper helper;

		int new_points;
        Map map;
		vector<vector<double>> current_trajectory;
		Road road;
		Vehicle my_car;
		int subpath_size;
		double duration;

		vector<STATES> possible_next_states();
		vector<vector<double>> state_transition(vector<STATES> possible_next_states);
		vector<vector<double>> generate_trajectory(STATES state);
		vector<double> get_kinematics(LANES lane);
		vector<vector<double>> keep_lane();
		vector<vector<double>> lane_change(STATES state);
		vector<vector<double>> prep_lane_change(STATES state);
		vector<vector<double>> create_JMT(vector<vector<double>> target_s_and_d);
		vector<vector<double>> produce_path(vector<double> target_s, vector<double> target_d);
		
    public:
	
		Path_Planner();
		~Path_Planner(){};

        vector<vector<double>> get_new_trajectory(Map &map, Road &road, Vehicle &my_car, vector<vector<double>> trajectory, int prev_path_size);


};



#endif
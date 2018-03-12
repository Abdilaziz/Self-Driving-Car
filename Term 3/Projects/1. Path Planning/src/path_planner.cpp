#include "path_planner.h"


Path_Planner::Path_Planner() {
	this->state = STATES::KEEP_LANE;
}

vector<double> JMT(vector< double> start, vector <double> end, double T) {
	
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    
    Eigen::MatrixXd A(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			    3*T*T, 4*T*T*T,5*T*T*T*T,
			    6*T, 12*T*T, 20*T*T*T;
		
	Eigen::MatrixXd B(3,1);	    
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
			    end[1]-(start[1]+start[2]*T),
			    end[2]-start[2];
			    
	Eigen::MatrixXd Ai = A.inverse();
	
	Eigen::MatrixXd C = Ai*B;
	
	vector <double> result = {start[0], start[1], .5*start[2]};
	for(int i = 0; i < C.size(); i++)
	{
	    result.push_back(C.data()[i]);
	}
	
    return result;
    
}

vector<vector<double>> Path_Planner::get_new_trajectory(Map &map, Road &road, Vehicle &my_car, vector<vector<double>> trajectory, int subpath_size) {

	this->map = map;
	this->current_trajectory = trajectory;
	this->my_car = my_car;
	this->road = road;
	this->subpath_size = subpath_size;
	this->new_points =  POINTS_PER_TRAJECTORY - subpath_size;
	this->duration = this->new_points*DELTA_T;

	vector<vector<double>> new_trajectory;
	vector<STATES> possible_new_state = possible_next_states();
	new_trajectory = state_transition(possible_new_state);

	return new_trajectory;
}

vector<STATES> Path_Planner::possible_next_states() {

	vector<STATES> states;
	states.push_back(STATES::KEEP_LANE);

	STATES state = this->state;
	if(state == STATES::KEEP_LANE) {
		states.push_back(STATES::PREPARE_CHANGE_LEFT);
		states.push_back(STATES::PREPARE_CHANGE_RIGHT);
	}
	else if (state == STATES::PREPARE_CHANGE_LEFT ) {
		if (this->my_car.getLane() != LANES::LEFT ) {
			states.push_back(STATES::PREPARE_CHANGE_LEFT);
			states.push_back(STATES::CHANGE_LEFT);
		}
	}
	else if (state == STATES::PREPARE_CHANGE_RIGHT) {
		if (this->my_car.getLane() != LANES::RIGHT ) {
			states.push_back(STATES::PREPARE_CHANGE_RIGHT);
			states.push_back(STATES::CHANGE_RIGHT);
		}
	} else if ( state == STATES::CHANGE_RIGHT) {
		if (this->my_car.getLane() != LANES::RIGHT ) {
			states.push_back(STATES::CHANGE_RIGHT);
		}
	} else if ( state == STATES::CHANGE_LEFT ) {
		if (this->my_car.getLane() != LANES::LEFT ) {
			states.push_back(STATES::CHANGE_LEFT);
		}
	}

	return states;
}

vector<vector<double>> Path_Planner::state_transition(vector<STATES> possible_next_states) {

	vector<vector<double>> new_trajectory;
	vector< vector<vector<double>>> final_trajectories;
	vector<STATES> states_tested;
	vector<double> costs;
	double cost;
	Costs costFunctions;

	for (vector<STATES>::iterator it = possible_next_states.begin(); it != possible_next_states.end(); ++it) {
		// cout << helper.state_to_string(*it) << endl;
		vector<vector<double>> target_s_and_d = generate_trajectory(*it);
		
		// cout << target_s_and_d[0][0] << "," << target_s_and_d[0][1] << endl;
		// cout << target_s_and_d[1][0] << "," << target_s_and_d[1][1] << endl;
		// cout << target_s_and_d[2][0] << "," << target_s_and_d[2][1] << endl;
		// cout << target_s_and_d[3][0] << "," << target_s_and_d[3][1] << endl;

		vector<vector<double>> trajectory = create_JMT(target_s_and_d);
		cost = costFunctions.calculate_cost(trajectory, this->road.getPredictions(), *it);
		costs.push_back(cost);
		final_trajectories.push_back(target_s_and_d);
		states_tested.push_back(*it);
		// cout << endl;
	}

	vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
	int best_idx = distance(begin(costs), best_cost);

	vector<vector<double>> target = final_trajectories[best_idx];
	this->state = states_tested[best_idx];

	// cout << "State Chosen: " << helper.state_to_string(states_tested[best_idx]) << endl;
	// cout << endl;
	// cout << "NEXT LOOP!!!" << endl;
	// cout << endl;

	new_trajectory = produce_path(target[1], target[3]);


	return new_trajectory;

}

vector<vector<double>> Path_Planner::generate_trajectory(STATES state) {

	vector<vector<double>> target_s_and_d;

	if (state == STATES::KEEP_LANE) {
		target_s_and_d = keep_lane();
	}
	else if (state == STATES::CHANGE_LEFT || state == STATES::CHANGE_RIGHT ) {
		target_s_and_d = lane_change(state);
	}
	else if (state == STATES::PREPARE_CHANGE_LEFT || state == STATES::PREPARE_CHANGE_RIGHT ) {
		target_s_and_d = prep_lane_change(state);
	}

	return target_s_and_d;
}

vector<double> Path_Planner::get_kinematics(LANES lane) {

	double old_velocity = this->my_car.getS_D();
	double old_position = this->my_car.getS();
	double old_accel = this->my_car.getS_DD();

	double new_velocity;
	double new_accel;
	double new_position;

	double max_velocity_accel_limit = old_velocity + MAX_INSTANTANEOUS_ACCEL/4 * this->duration;

	if (this->road.isVehicleAhead(old_position, lane)) {
		// cout << "VEHICLE AHEAD: " << helper.lane_to_string(lane) << endl;
		if (this->road.isVehicleBehind(old_position, lane)) {
			// cout << "Vehicle Behind: " << helper.lane_to_string(lane) << endl;
			new_velocity = this->road.getVehicleAhead().getS_D(); //must travel at the speed of traffic, regardless of preferred buffer
		}
		else {
			double max_velocity_in_front = max( old_velocity - (MAX_INSTANTANEOUS_ACCEL/4*this->duration) , this->road.getVehicleAhead().getS_D());   // (this->road.getVehicleAhead().getS() - old_position - 30 ) + this->road.getVehicleAhead().getS_D() - 0.5*(old_accel);
			max_velocity_in_front -= 2; // a little slower then the vehicle ahead to gain distance
			new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit),SPEED_LIMIT );
		}	
	}
	else {
		// cout << "NO VEHICLE AHEAD: " << helper.lane_to_string(lane) << endl;
		new_velocity = min(max_velocity_accel_limit, SPEED_LIMIT);
	}
	// cout << "New Speed: " << new_velocity << endl;
	new_accel = (new_velocity - old_velocity)/this->duration; // Equation: (v_1 - v_0)/t = acceleration
	new_position = old_position + new_velocity*this->duration + 0.5*new_accel*(this->duration)*(this->duration);

	return {new_position, new_velocity, new_accel};
}

vector<vector<double>> Path_Planner::keep_lane() {

	double old_velocity = this->my_car.getS_D();
	double old_position = this->my_car.getS();
	double old_accel = this->my_car.getS_DD();
	LANES old_lane = this->my_car.getLane();

	vector<double> kinematics = get_kinematics(old_lane);
	double new_position = kinematics[0];
	double new_velocity = kinematics[1];
	double new_accel = kinematics[2];


	vector<double> start_s = {old_position, old_velocity, old_accel};
	vector<double> end_s = {new_position, new_velocity, new_accel};

	vector<double> start_d = { this->my_car.getD(), this->my_car.getD_D(), this->my_car.getD_DD() };
	vector<double> end_d = { helper.lane_to_d( old_lane ) , 0.0, 0.0 };

	return { start_s, end_s, start_d, end_d };
}

vector<vector<double>> Path_Planner::lane_change(STATES state) {
	
	LANES newLane = helper.changeLane(state, this->my_car.getLane());
	
	double old_velocity = this->my_car.getS_D();
	double old_position = this->my_car.getS();
	double old_accel = this->my_car.getS_DD();

	vector<double> kinematics = get_kinematics(newLane);
	double new_position = kinematics[0];
	double new_velocity = kinematics[1];
	double new_accel = kinematics[2];


	vector<double> start_s = {old_position, old_velocity, old_accel};
	vector<double> end_s = {new_position, new_velocity, new_accel};

	vector<double> start_d = { this->my_car.getD(), this->my_car.getD_D(), this->my_car.getD_DD() };
	vector<double> end_d = { helper.lane_to_d( newLane ) , 0.0, 0.0 };

	return { start_s, end_s, start_d, end_d };
}

vector<vector<double>> Path_Planner::prep_lane_change(STATES state) {

	LANES newLane = helper.changeLane(state, this->my_car.getLane());

	double old_velocity = this->my_car.getS_D();
	double old_position = this->my_car.getS();
	double old_accel = this->my_car.getS_DD();
	LANES old_lane = this->my_car.getLane();

	vector<double> cur_lane_kinematics = get_kinematics(old_lane);
	double new_position, new_velocity, new_accel;

	if (this->road.isVehicleBehind(old_position, old_lane)) {
		new_position = cur_lane_kinematics[0];
		new_velocity = cur_lane_kinematics[1];
		new_accel = cur_lane_kinematics[2];
	}
	else {
		// if there is no vehicle behind you, match the speed of the other lane if it is slower than the speed of the current one
		vector<double> best_kinematics;
		vector<double> desired_lane_kinematics = get_kinematics(newLane);
		
		if (desired_lane_kinematics[1] < cur_lane_kinematics[1] ) {
			best_kinematics = desired_lane_kinematics;
		}
		else {
			best_kinematics = cur_lane_kinematics;
		}

		new_position = best_kinematics[0];
		new_velocity = best_kinematics[1];
		new_accel = best_kinematics[2];
	}

	vector<double> start_s = {old_position, old_velocity, old_accel};
	vector<double> end_s = {new_position, new_velocity, new_accel};

	vector<double> start_d = { this->my_car.getD(), this->my_car.getD_D(), this->my_car.getD_DD() };
	vector<double> end_d = { helper.lane_to_d( old_lane ) , 0.0, 0.0 };

	return { start_s, end_s, start_d, end_d };
}

vector<vector<double>> Path_Planner::create_JMT(vector<vector<double>> target_s_and_d) {

	vector<double> start_s = target_s_and_d[0];
	vector<double> end_s = target_s_and_d[1];
	vector<double> start_d = target_s_and_d[2];
	vector<double> end_d = target_s_and_d[3];

	vector<double> poly_s = JMT(start_s, end_s, this->duration);
	vector<double> poly_d = JMT(start_d, end_d, this->duration);

	vector<double> new_trajectory_s;
	vector<double> new_trajectory_d;

	double t, next_s, next_d, mod_s, mod_d;

	vector <double> XY;
	for(int i = 0; i < this->new_points; i++) {

		t = i * DELTA_T; // duration/POINTS_TO_TARGET;


		next_s = 0.0;
		next_d = 0.0;
		for (int a = 0; a < poly_s.size(); a++) {
		  next_s += poly_s[a] * pow(t, a);
		  next_d += poly_d[a] * pow(t, a);
		}

		mod_s = fmod(next_s, MAX_S);
		mod_d = fmod(next_d, LANE_WIDTH*3);

		new_trajectory_s.push_back(mod_s);

		new_trajectory_d.push_back(mod_d);
	}

	return {new_trajectory_s, new_trajectory_d};

}

vector<vector<double>> Path_Planner::produce_path(vector<double> target_s, vector<double> target_d) {

	//cout <<  "Producing Path" << endl;

	vector<double> spline_points_s, spline_points_x, spline_points_y, new_s_traj, 
												 new_x_traj, new_y_traj;

	double pos_s = this->my_car.getS();
	double pos_x = this->my_car.getX();
	double pos_y = this->my_car.getY();
	double s_dot = this->my_car.getS_D();
	double s_ddot = this->my_car.getS_DD();


	double angle = this->my_car.getYAW();


	double prev_s = pos_s - s_dot * DELTA_T;
					
	// first two points of coarse trajectory, to ensure spline begins smoothly
	if (this->subpath_size >= 2) {
		spline_points_s.push_back(prev_s);
		spline_points_x.push_back(this->current_trajectory[0][this->subpath_size-2]);
		spline_points_y.push_back(this->current_trajectory[1][this->subpath_size-2]);
		spline_points_s.push_back(pos_s);
		spline_points_x.push_back(this->current_trajectory[0][this->subpath_size-1]);
		spline_points_y.push_back(this->current_trajectory[1][this->subpath_size-1]);
	} else {
		double prev_s = pos_s - 1;
		double prev_x = pos_x - cos(angle);
		double prev_y = pos_y - sin(angle);
		spline_points_s.push_back(prev_s);
		spline_points_x.push_back(prev_x);
		spline_points_y.push_back(prev_y);
		spline_points_s.push_back(pos_s);
		spline_points_x.push_back(pos_x);
		spline_points_y.push_back(pos_y);
	}

	double s1 = pos_s + 30;
	double d1 = target_d[0];
	vector<double> xy1 = this->map.getXY(s1, d1);
	double x1 = xy1[0];
	double y1 = xy1[1];
	spline_points_s.push_back(s1);
	spline_points_x.push_back(x1);
	spline_points_y.push_back(y1);
	double s2 = s1 + 30;
	double d2 = d1;
	vector<double> xy2 = this->map.getXY(s2, d2);
	double x2 = xy2[0];
	double y2 = xy2[1];
	spline_points_s.push_back(s2);
	spline_points_x.push_back(x2);
	spline_points_y.push_back(y2);

	// //cout << "Current S: " << pos_s << endl;
	// //cout << "Current V: " << s_dot << endl;
	// //cout << "Current A: " << s_ddot << endl;

	double target_s_dot = target_s[1];

	double current_s = pos_s;
	double current_v = s_dot;
	double current_a = s_ddot;
	for (int i = 0; i < (POINTS_PER_TRAJECTORY - this->subpath_size); i++) {
		double v_incr, a_incr;
		if (fabs(target_s_dot - current_v) < 2 * VELOCITY_INCREMENT_LIMIT) {
			v_incr = 0;
		} else {

			v_incr = (target_s_dot - current_v)/(fabs(target_s_dot - current_v)) * VELOCITY_INCREMENT_LIMIT;
		}
		current_v += v_incr;
		current_s += current_v * DELTA_T;
		new_s_traj.push_back(current_s);
	}

	new_x_traj = this->map.interpolate_points(spline_points_s, spline_points_x, new_s_traj);
	new_y_traj = this->map.interpolate_points(spline_points_s, spline_points_y, new_s_traj);



	vector<double> new_trajectory_x;
	vector<double> new_trajectory_y;

	for(int i = 0; i < this->subpath_size; i++) {
		new_trajectory_x.push_back(this->current_trajectory[0][i]);
		new_trajectory_y.push_back(this->current_trajectory[1][i]);
	} 
	for (int i = 0; i < new_x_traj.size(); i++) {
		new_trajectory_x.push_back(new_x_traj[i]);
		new_trajectory_y.push_back(new_y_traj[i]);
	} 

	return {new_trajectory_x, new_trajectory_y};

}
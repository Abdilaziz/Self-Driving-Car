#include "path_planning.h"


Path_Planner::Path_Planner() {
	//this->state = STATES::START;
	this->state = STATES::KEEP_LANE;
}

string lane_to_string(LANES lane){
  if (lane == LANES::LEFT){
    return "LEFT";
  } else if(lane == LANES::CENTER){
    return "CENTER";
  } else{
    return "RIGHT";
  }
}

string state_to_string(STATES state){
  if (state == STATES::KEEP_LANE){
    return "KEEP_LANE";
  } 
  else if (state == STATES::PREPARE_CHANGE_LEFT ) {
  	return "PREPARE_CHANGE_LEFT";
  }
  else if (state == STATES::PREPARE_CHANGE_RIGHT ){
  	return "PREPARE_CHANGE_RIGHT";
  }
  else if(state == STATES::CHANGE_LEFT){
    return "CHANGE_LEFT";
  } 
  else if(state == STATES::START){
    return "START";
  } 
  else{
    return "CHANGE_RIGHT";
  }
}

vector<double> Path_Planner::JMT(vector< double> start, vector <double> end, double T) {
	
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


double Path_Planner::lane_to_d(LANES lane) {
	return ((double) lane)*LANE_WIDTH + 2.0;
}

LANES Path_Planner::d_to_lane(double d) {
	LANES lane;
	if (d > 0 && d < LANE_WIDTH) {
		lane = LANES::LEFT;
	}
	else if (d >= LANE_WIDTH  &&  d < 2*LANE_WIDTH) {
		lane = LANES::CENTER;
	}
	else if (d >= 2*LANE_WIDTH && d <= 3*LANE_WIDTH ) {
		lane = LANES::RIGHT;
	}
	return lane;
}


vector<STATES> Path_Planner::possible_next_states() {

	STATES cur_state = this->state;
	LANES cur_lane = this->my_car.getLane();

	vector<STATES> possible_new_state;
	possible_new_state.push_back(STATES::KEEP_LANE);



	bool car_ahead = this->road.isVehicleAhead(this->my_car, cur_lane);
	bool car_left = true;
	bool car_right = true;
	int left_lane = (((int) cur_lane)-1);
	int right_lane = (((int) cur_lane) +1); 
	if (left_lane >= 0) {
		car_left = !(this->road.isLaneChangeSafe(this->my_car,(LANES) left_lane));
		//cout << "Car on left!!!!!!!!!!!" << endl;
	}
	if (right_lane <= 2) {
		car_right = !(this->road.isLaneChangeSafe(this->my_car,(LANES) right_lane));
		//cout << "Car on right!!!!!!!!!!" << endl;
	}

	if (cur_state == STATES::KEEP_LANE) {
		if (car_ahead ) {
 			possible_new_state.push_back(STATES::PREPARE_CHANGE_LEFT);
 			possible_new_state.push_back(STATES::PREPARE_CHANGE_RIGHT);
		}
	}
	else if (cur_state == STATES::PREPARE_CHANGE_LEFT) {
		 	possible_new_state.push_back(STATES::PREPARE_CHANGE_RIGHT);
		 	possible_new_state.push_back(STATES::CHANGE_LEFT);
	}
	else if (cur_state == STATES::PREPARE_CHANGE_RIGHT) {
		
		 	possible_new_state.push_back(STATES::PREPARE_CHANGE_LEFT);
		 	possible_new_state.push_back(STATES::CHANGE_RIGHT);
	}
	else if (cur_state == STATES::CHANGE_RIGHT) {
		possible_new_state.push_back(STATES::CHANGE_RIGHT);
	}
	else if (cur_state == STATES::CHANGE_LEFT ) {
		possible_new_state.push_back(STATES::CHANGE_LEFT);
	}

	return possible_new_state;
}


vector<vector<double>> Path_Planner::state_transition(vector<STATES> possible_next_states) {

	vector<vector<double>> new_trajectory;

	vector <double> costs;
	vector< vector<vector<double>>> final_trajectories;

	vector<STATES> states_tested;

	
	
	for (vector<STATES>::iterator it = possible_next_states.begin(); it!=possible_next_states.end(); ++it ) {
		
		//cout << "State: " << state_to_string(*it) << " ";
		Costs costFunctions;
		vector<vector<double>> target_s_and_d =  this->get_target(*it);
		//cout << endl;
		//cout << "End S: ";

		// for ( auto i = target_s_and_d[1].begin(); i != target_s_and_d[1].end(); ++i ) {
		// 	//cout << *i << " ";
		// }
		// //cout << endl;

		// //cout << "End D: ";

		// for ( auto i = target_s_and_d[3].begin(); i != target_s_and_d[3].end(); ++i ) {
		// 	//cout << *i << " ";
		// }
		//cout << endl;

		vector<vector<double>> trajectory = create_JMT(target_s_and_d[0], target_s_and_d[1], target_s_and_d[2], target_s_and_d[3], this->duration );
		double cost = costFunctions.calculate_total_cost(trajectory[0], trajectory[1], this->road.getPredictions());
		costs.push_back(cost);
		final_trajectories.push_back(target_s_and_d);
		states_tested.push_back(*it);
		//cout << "Cost: " << cost << endl;
	}

	//cout << endl;

	vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
	int best_idx = distance(begin(costs), best_cost);

	vector<vector<double>> target = final_trajectories[best_idx];

	//cout << "State Choosen: " << state_to_string(states_tested[best_idx]) << endl;

	this->state = states_tested[best_idx];

	new_trajectory = proeduce_path(target[1], target[3]);


	return new_trajectory;
}

vector<vector<double>> Path_Planner::basic_state_transition() {

	vector<vector<double>> target_s_and_d;
	LANES my_lane = this->my_car.getLane();

		if (this->state == STATES::START ) {
			target_s_and_d = this->startVehicle();
			this->state = STATES::KEEP_LANE;
		}
		else if ( this->state == STATES::KEEP_LANE ) {

			if (this->road.isVehicleAhead(this->my_car, my_lane)) {

				//cout<< "vehicle ahead " << endl;
				LANES target_lane =  this->road.availableLane(this->my_car);
				if (target_lane == my_lane) {
					// nothing available
					target_s_and_d = this->slow_down();
					this->state = STATES::KEEP_LANE;
				}
				else {

					target_s_and_d = this->change_lane(target_lane);
					if (target_lane == LANES::LEFT){
						this->state = STATES::CHANGE_LEFT;
					}
					else if (target_lane == LANES::RIGHT) {
						this->state = STATES::CHANGE_RIGHT;
					}
					else {
						if (my_lane == LANES::LEFT) {
							this->state = STATES::CHANGE_RIGHT;
						}
						else {
							this->state = STATES::CHANGE_LEFT;
						}
					}
					
				}

			}
			else {
				target_s_and_d = this->keep_in_lane();
				this->state = STATES::KEEP_LANE;
			}
		}
		else {
			if (this->road.isVehicleAhead( this->my_car , my_lane)) {
				target_s_and_d = this->slow_down();
				this->state = STATES::KEEP_LANE;
			}
			else {
				target_s_and_d = this->keep_in_lane();
				this->state = STATES::KEEP_LANE;
			}


		}
	//}
	// else {
	// 	target_s_and_d = this->trajectory;
	// }

	vector<vector<double>> new_trajectory = create_JMT(target_s_and_d[0], target_s_and_d[1], target_s_and_d[2], target_s_and_d[3], this->duration );
	vector<vector<double>> new_path = proeduce_path(target_s_and_d[1], target_s_and_d[3]);

	return new_path;
}



vector<vector<double>> Path_Planner::get_target(STATES state) {

	vector<vector<double>> target_s_and_d;

	if (state == STATES::START) {
		target_s_and_d = this->startVehicle();
	}
	else if (state == STATES::KEEP_LANE) {
		target_s_and_d = this->keep_in_lane();
	}
	else if (state == STATES::PREPARE_CHANGE_LEFT || state == STATES::PREPARE_CHANGE_RIGHT ) {
		target_s_and_d = this->slow_down();
	}
	else if (state == STATES::CHANGE_LEFT || state == STATES::CHANGE_RIGHT) {
		target_s_and_d = this->change_lane(state);
	}

	return target_s_and_d;
}


vector<vector<double>> Path_Planner::get_new_trajectory(Map &map, Road &road, Vehicle &my_car, vector<vector<double>> trajectory, int prev_path_size) {

	this->map = map;
	this->current_trajectory = trajectory;
	this->my_car = my_car;
	this->road = road;
	this->prev_path_size = prev_path_size;
	this->new_points =  POINTS_PER_TRAJECTORY - prev_path_size;
	this->duration = POINTS_TO_TARGET*DT - prev_path_size*DELTA_T;
	//cout << "NEW POINTS: " << this->new_points << endl;

	vector<vector<double>> new_trajectory;
	vector<STATES> possible_new_state;
	possible_new_state = this->possible_next_states();
	new_trajectory = this->state_transition(possible_new_state);

	return new_trajectory;
}


vector<vector<double>> Path_Planner::startVehicle() {

	//cout << "Start trajectory" << endl;


	double target_velocity = MAX_INSTANTANEOUS_ACCEL/4 * duration;


	double target_s = this->my_car.getS() + this->duration * target_velocity;

	vector<double> start_s;
	vector<double> end_s;

	vector<double> start_d;
	vector<double> end_d;

	start_s = { this->my_car.getS(), this->my_car.getS_D(), this->my_car.getS_DD() };

	end_s = {target_s, target_velocity, 0.0};

	start_d = { this->my_car.getD(), this->my_car.getD_D(), this->my_car.getD_DD() };

	end_d = {this->lane_to_d(this->my_car.getLane()), 0.0, 0.0};

	return { start_s, end_s, start_d, end_d };
}

vector<vector<double>> Path_Planner::keep_in_lane() {

	//cout << "KEEP in lane trajectory ";

	double old_velocity = this->my_car.getS_D();
	double old_position = this->my_car.getS();

	double velocity_car_ahead = this->road.getVehicleAhead().getS_D();


	double target_velocity;

	target_velocity = min(old_velocity+ MAX_INSTANTANEOUS_ACCEL/4 * this->duration, SPEED_LIMIT);

	double target_s = old_position + this->duration * target_velocity;

	vector<double> start_s;
	vector<double> end_s;

	vector<double> start_d;
	vector<double> end_d;

	start_s = { this->my_car.getS(), this->my_car.getS_D(), this->my_car.getS_DD() };

	end_s = {target_s, target_velocity, 0.0};

	start_d = { this->my_car.getD(), this->my_car.getD_D(), this->my_car.getD_DD() };

	end_d = {this->lane_to_d( this->my_car.getLane() ) , 0.0, 0.0};


	return { start_s, end_s, start_d, end_d };
}


vector<vector<double>> Path_Planner::slow_down() {

	//cout << "slow down trajectory ";

	double position_car_ahead = this->road.getVehicleAhead().getS();
	double velocity_car_ahead = this->road.getVehicleAhead().getS_D();

	double old_velocity = this->my_car.getS_D();
	double old_position = this->my_car.getS();

	double target_velocity;
	double target_s;

	target_velocity = max(old_velocity - MAX_INSTANTANEOUS_ACCEL/4 * this->duration, velocity_car_ahead);
	if (position_car_ahead > 0 ) {
		target_s = position_car_ahead - 10;
	}
	else {
		target_s = old_position + target_velocity*this->duration;
	}


	vector<double> start_s;
	vector<double> end_s;

	vector<double> start_d;
	vector<double> end_d;


	start_s = { this->my_car.getS(), this->my_car.getS_D(), this->my_car.getS_DD() };
	end_s = { target_s, target_velocity, 0.0 };

	start_d = { this->my_car.getD(), this->my_car.getD_D(), this->my_car.getD_DD() };

	end_d = {this->lane_to_d( this->my_car.getLane()) , 0.0, 0.0};

	return {  start_s, end_s, start_d, end_d };
}

vector<vector<double>> Path_Planner::change_lane(STATES state) {

	//cout << "change lane trajectory ";

	LANES target_lane;

	if (this->my_car.getLane() == LANES::LEFT) {
		if (state == STATES::CHANGE_RIGHT) {
			target_lane = LANES::CENTER;
		}
	}
	else if (this->my_car.getLane() == LANES::CENTER) {
		if (state == STATES::CHANGE_LEFT) {
			target_lane = LANES::LEFT;
		}
		else {
			target_lane = LANES::RIGHT;
		}
	}
	else {
		if (state == STATES::CHANGE_LEFT) {
			target_lane = LANES::CENTER;
		}
	}

	//cout << "change lane State: " << state_to_string(state) << endl;
	//cout << "change lane Target Lane: " << lane_to_string(target_lane) << endl;

	double old_velocity = this->my_car.getS_D();
	double old_position = this->my_car.getS();

	double target_velocity = old_velocity;
	double target_s = old_position + this->duration * target_velocity;

	vector<double> start_s;
	vector<double> end_s;

	vector<double> start_d;
	vector<double> end_d;

	start_s = { this->my_car.getS(), this->my_car.getS_D(), this->my_car.getS_DD() };
	end_s = {target_s, target_velocity, 0.0 };

	double target_d = this->lane_to_d(target_lane);

	start_d = { this->my_car.getD(), this->my_car.getD_D(), this->my_car.getD_DD() };

	end_d = { target_d , 0.0, 0.0 };

	return { start_s, end_s, start_d, end_d };
}

vector<vector<double>> Path_Planner::change_lane(LANES target_lane) {


	double old_velocity = this->my_car.getS_D();
	double old_position = this->my_car.getS();

	double target_velocity = old_velocity;
	double target_s = old_position + this->duration * target_velocity;

	vector<double> start_s;
	vector<double> end_s;

	vector<double> start_d;
	vector<double> end_d;

	start_s = { this->my_car.getS(), this->my_car.getS_D(), this->my_car.getS_DD() };
	end_s = {target_s, target_velocity, 0.0 };

	double target_d = this->lane_to_d(target_lane);

	start_d = { this->my_car.getD(), this->my_car.getD_D(), this->my_car.getD_DD() };

	end_d = { target_d , 0.0, 0.0 };

	return { start_s, end_s, start_d, end_d };
}

vector<vector<double>> Path_Planner::create_JMT(vector<double> start_s, vector<double> end_s, vector<double> start_d, vector<double> end_d, double duration){

	vector<double> poly_s = this->JMT(start_s, end_s, duration);
	vector<double> poly_d = this->JMT(start_d, end_d, duration);

	vector<double> new_trajectory_s;
	vector<double> new_trajectory_d;

	double t, next_s, next_d, mod_s, mod_d;

	vector <double> XY;
	for(int i = 0; i < POINTS_TO_TARGET; i++) {

		t = i * duration/POINTS_TO_TARGET;


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



vector<vector<double>> Path_Planner::proeduce_path(vector<double> target_s, vector<double> target_d) {

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
	if (this->prev_path_size >= 2) {
		spline_points_s.push_back(prev_s);
		spline_points_x.push_back(this->current_trajectory[0][this->prev_path_size-2]);
		spline_points_y.push_back(this->current_trajectory[1][this->prev_path_size-2]);
		spline_points_s.push_back(pos_s);
		spline_points_x.push_back(this->current_trajectory[0][this->prev_path_size-1]);
		spline_points_y.push_back(this->current_trajectory[1][this->prev_path_size-1]);
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
	for (int i = 0; i < (POINTS_PER_TRAJECTORY - this->prev_path_size); i++) {
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

	for(int i = 0; i < this->prev_path_size; i++) {
		new_trajectory_x.push_back(this->current_trajectory[0][i]);
		new_trajectory_y.push_back(this->current_trajectory[1][i]);
	} 
	for (int i = 0; i < new_x_traj.size(); i++) {
		new_trajectory_x.push_back(new_x_traj[i]);
		new_trajectory_y.push_back(new_y_traj[i]);
	} 

	return {new_trajectory_x, new_trajectory_y};

}


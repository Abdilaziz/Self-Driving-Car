#include "path_planning.h"


Path_Planner::Path_Planner() {
	this->state = STATES::START;
}

string slane(LANES lane){
  if (lane == LANES::LEFT){
    return "LEFT";
  } else if(lane == LANES::CENTER){
    return "CENTER";
  } else{
    return "RIGHT";
  }
}

string sstate(STATES state){
  if (state == STATES::KEEP_LANE){
    return "KEEP_LANE";
  } else if(state == STATES::CHANGE_LEFT){
    return "CHANGE_LEFT";
  } else if(state == STATES::START){
    return "START";
  } else{
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

	if (cur_state == STATES::KEEP_LANE) {

		possible_new_state.push_back(STATES::PREPARE_CHANGE_LEFT);
		possible_new_state.push_back(STATES::PREPARE_CHANGE_RIGHT);

	}
	else if (cur_state == STATES::PREPARE_CHANGE_LEFT) {
		if (cur_lane != LANES::LEFT) {
			possible_new_state.push_back(STATES::PREPARE_CHANGE_LEFT);
			possible_new_state.push_back(STATES::CHANGE_LEFT);
		}
	}
	else if (cur_state == STATES::PREPARE_CHANGE_RIGHT) {
		if (cur_lane != LANES::RIGHT) {
			possible_new_state.push_back(STATES::PREPARE_CHANGE_RIGHT);
			possible_new_state.push_back(STATES::CHANGE_RIGHT);
		}
	}

	return possible_new_state;
}


vector<vector<double>> Path_Planner::state_transition(vector<STATES> possible_next_states) {

	vector<vector<double>> new_trajectory;

	// vector <double> costs;
	// vector< vector<vector<double>>> final_trajectories;
	
	// for (vector<STATES>::iterator it = possible_next_states.begin(); it!=possible_next_states.end(); ++it ) {

	// 	vector<vector<double>> trajectory =  this->generate_trajectory(*it);
	// 	double cost = calculate_cost(this->my_car, this->road, trajectory);
	// 	costs.push_back(cost);
	// 	final_trajectories.push_back(trajectory);
	// }

	// vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
	// int best_idx = distance(begin(costs), best_cost);

	// new_trajectory = final_trajectories[best_idx];


	return new_trajectory;
}

vector<vector<double>> Path_Planner::basic_state_transition() {

	vector<vector<double>> new_trajectory;
	LANES my_lane = this->my_car.getLane();

	cout << "STATE: " << sstate(this->state) << endl;
	cout << "LANE: " << slane(my_lane) << endl;


	if (this->prev_path_size < POINTS_PER_TRAJECTORY) {

		if (this->state == STATES::START ) {
			new_trajectory = this->startVehicle();
			this->state = STATES::KEEP_LANE;
		}
		else if ( this->state == STATES::KEEP_LANE ) {

			//new_trajectory = this->keep_in_lane();

			if (this->road.isVehicleAhead(this->my_car, my_lane)) {

				cout<< "vehicle ahead " << endl;
				//LANES target_lane =  this->road.availableLane(this->my_car);
				//if (target_lane == my_lane) {
					// nothing available
					new_trajectory = this->slow_down();
					this->state = STATES::KEEP_LANE;
				// }
				// else {

					// new_trajectory = this->change_lane(target_lane);
					// if (target_lane == LANES::LEFT){
					// 	this->state = STATES::CHANGE_LEFT;
					// }
					// else if (target_lane == LANES::RIGHT) {
					// 	this->state = STATES::CHANGE_RIGHT;
					// }
					// else {
					// 	if (my_lane == LANES::LEFT) {
					// 		this->state = STATES::CHANGE_RIGHT;
					// 	}
					// 	else {
					// 		this->state = STATES::CHANGE_LEFT;
					// 	}
					// }
					
				//}

			}
			else {
				new_trajectory = this->keep_in_lane();
				this->state = STATES::KEEP_LANE;
			}
		}
		else {
			if (this->road.isVehicleAhead( this->my_car , my_lane)) {
				new_trajectory = this->slow_down();
				this->state = STATES::KEEP_LANE;
			}
			else {
				new_trajectory = this->keep_in_lane();
				this->state = STATES::KEEP_LANE;
			}


		}
	}
	else {
		new_trajectory = this->trajectory;
	}

	return new_trajectory;
}






vector<vector<double>> Path_Planner::generate_trajectory(STATES state) {

	vector<vector<double>> new_trajectory;

	if (state == STATES::START) {
		new_trajectory = this->startVehicle();
	}
	else if (state == STATES::KEEP_LANE) {
		new_trajectory = this->keep_in_lane();
	}
	else if (state == STATES::PREPARE_CHANGE_LEFT || state == STATES::PREPARE_CHANGE_RIGHT ) {
		new_trajectory = this->slow_down();
	}
	else if (state == STATES::CHANGE_LEFT || state == STATES::CHANGE_RIGHT) {
		new_trajectory = this->change_lane(state);
	}

	return new_trajectory;
}


vector<vector<double>> Path_Planner::get_new_trajectory(Map &map, Road &road, Vehicle &my_car, vector<vector<double>> trajectory, int prev_path_size, vector<vector<double>> points_for_spline) {

	this->map = map;

	this->trajectory = trajectory;


	this->my_car = my_car;
	this->road = road;
	this->prev_path_size = prev_path_size;
	this->points_for_spline = points_for_spline;

	this->points_in_horizon =  POINTS_PER_TRAJECTORY - this->prev_path_size;

	vector<vector<double>> new_trajectory;

	// vector<vector<double>> updated_trajectory;
	// vector<STATES> possible_new_state;

	// if ( prev_path_size < POINTS_PER_TRAJECTORY ) {


	// 	possible_new_state = this->possible_next_states();

	// 	new_trajectory = this->state_transition(possible_next_states);

	// }

	// BASIC STATES TRANSITION

	new_trajectory = this->basic_state_transition();



	return new_trajectory;
}


vector<vector<double>> Path_Planner::startVehicle() {

	cout << "Start trajectory" << endl;

	//this->points_in_horizon = 4*POINTS_PER_TRAJECTORY;
	double duration = POINTS_TO_TARGET * DT - this->prev_path_size * DELTA_T;

	double target_velocity = SPEED_LIMIT/2;


	double target_s = this->my_car.getS() + duration * target_velocity;

	vector<double> start_s;
	vector<double> end_s;

	vector<double> start_d;
	vector<double> end_d;

	start_s = { this->my_car.getS(), this->my_car.getV(), 0.0 };

	end_s = {target_s, target_velocity, 0.0};

	start_d = {this->lane_to_d(this->my_car.getLane()), 0.0, 0.0};

	end_d = {this->lane_to_d(this->my_car.getLane()), 0.0, 0.0};

	return this->create_JMT(start_s, end_s, start_d, end_d, duration);
	//return { { start_s, end_s}, {start_d, end_d} };
}

vector<vector<double>> Path_Planner::keep_in_lane() {

	cout << "KEEP in lane trajectory" << endl;

	//this->points_in_horizon = 2*POINTS_PER_TRAJECTORY;
	double duration = POINTS_TO_TARGET * DT - this->prev_path_size * DELTA_T;

	double old_velocity = this->my_car.getV();
	double old_position = this->my_car.getS();

	double target_velocity = min(old_velocity*1.2, SPEED_LIMIT);

	double target_s = old_position + duration * target_velocity;


	vector<double> start_s;
	vector<double> end_s;

	vector<double> start_d;
	vector<double> end_d;

	start_s = { old_position, old_velocity, 0.0};

	end_s = {target_s, target_velocity, 0.0};

	start_d = {this->my_car.getD() , 0.0, 0.0};

	end_d = {this->lane_to_d( this->my_car.getLane() ) , 0.0, 0.0};


	return this->create_JMT(start_s, end_s, start_d, end_d, duration);
	//return { { start_s, end_s}, {start_d, end_d} };
}


vector<vector<double>> Path_Planner::slow_down() {

	cout << "slow down trajectory" << endl;

	//this->points_in_horizon = 2*POINTS_PER_TRAJECTORY;
	double duration = POINTS_TO_TARGET * DT - this->prev_path_size * DELTA_T;

	double velocity_car_ahead = this->road.getSPEED();

	double old_velocity = this->my_car.getV();
	double old_position = this->my_car.getS();

	//double target_velocity = max(old_velocity*0.75, SPEED_LIMIT/2);
	double target_velocity = velocity_car_ahead;
	double target_s = old_position + duration * target_velocity;

	vector<double> start_s;
	vector<double> end_s;

	vector<double> start_d;
	vector<double> end_d;


	start_s = { old_position, old_velocity, 0.0 };
	end_s = { target_s, target_velocity, 0.0 };

	start_d = {this->my_car.getD() , 0.0, 0.0};

	end_d = {this->lane_to_d( this->my_car.getLane() ) , 0.0, 0.0};

	return create_JMT(start_s, end_s, start_d, end_d, duration);
	//return { { start_s, end_s}, {start_d, end_d} };
}

vector<vector<double>> Path_Planner::change_lane(STATES state) {

	cout << "change lane trajectory" << endl;

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

	//this->points_in_horizon = 2*POINTS_PER_TRAJECTORY;
	double duration = POINTS_TO_TARGET * DT - this->prev_path_size * DELTA_T;

	double old_velocity = this->my_car.getV();
	double old_position = this->my_car.getS();

	double target_velocity = old_velocity;
	double target_s = old_position + duration * target_velocity;

	vector<double> start_s;
	vector<double> end_s;

	vector<double> start_d;
	vector<double> end_d;

	start_s = { old_position, old_velocity, 0.0 };
	end_s = {target_s, target_velocity, 0.0 };

	double target_d = this->lane_to_d(target_lane);

	start_d = {this->my_car.getD() , 0.0, 0.0};
	end_d = { target_d , 0.0, 0.0 };

	return create_JMT(start_s, end_s, start_d, end_d, duration);
	//return { { start_s, end_s}, {start_d, end_d} };
}

vector<vector<double>> Path_Planner::change_lane(LANES target_lane) {


	//this->points_in_horizon = 2*POINTS_PER_TRAJECTORY;
	double duration = POINTS_TO_TARGET * DT - this->prev_path_size * DELTA_T;

	double old_velocity = this->my_car.getV();
	double old_position = this->my_car.getS();

	double target_velocity = old_velocity;
	double target_s = old_position + duration * target_velocity;

	vector<double> start_s;
	vector<double> end_s;

	vector<double> start_d;
	vector<double> end_d;

	start_s = { old_position, old_velocity, 0.0 };
	end_s = {target_s, target_velocity, 0.0 };

	double target_d = this->lane_to_d(target_lane);

	start_d = {this->my_car.getD() , 0.0, 0.0};
	end_d = { target_d , 0.0, 0.0 };

	return create_JMT(start_s, end_s, start_d, end_d, duration);
	//return { { start_s, end_s}, {start_d, end_d} };
}

vector<vector<double>> Path_Planner::create_JMT(vector<double> start_s, vector<double> end_s, vector<double> start_d, vector<double> end_d, double duration){

	cout << "Start S: ";

	for ( auto i = start_s.begin(); i != start_s.end(); ++i ) {
		cout << *i << " ";
	}
	cout << endl;

	cout << "End S: ";

	for ( auto i = end_s.begin(); i != end_s.end(); ++i ) {
		cout << *i << " ";
	}
	cout << endl;

	cout << "Start D: ";

	for ( auto i = start_d.begin(); i != start_d.end(); ++i ) {
		cout << *i << " ";
	}
	cout << endl;

	cout << "End D: ";

	for ( auto i = end_d.begin(); i != end_d.end(); ++i ) {
		cout << *i << " ";
	}
	cout << endl;

	
	//double T = this->points_in_horizon * DELTA_T;

	//double T =   POINTS_TO_TARGET * DELTA_T;

	cout << "Horizon: " << duration << endl;

	vector<double> poly_s = this->JMT(start_s, end_s, duration);
	vector<double> poly_d = this->JMT(start_d, end_d, duration);

	double t, next_s, next_d, mod_s, mod_d;
	vector <double> XY;
	for(int i = 1; i <= POINTS_TO_TARGET; i++) {

		//t = DELTA_T*i;
		t = i * duration/POINTS_TO_TARGET;


		next_s = 0.0;
		next_d = 0.0;
		for (int a = 0; a < poly_s.size(); a++) {
		  next_s += poly_s[a] * pow(t, a);
		  next_d += poly_d[a] * pow(t, a);
		}

		mod_s = fmod(next_s, MAX_S);
		mod_d = fmod(next_d, LANE_WIDTH*3);

		XY = map.getXY(mod_s + i*30, mod_d);

		this->points_for_spline[0].push_back(XY[0]);
		this->points_for_spline[1].push_back(XY[1]);
	}


	// tk::spline s;
	// s.set_points(new_trajectory[0], new_trajectory[1]);
	// for (int i = 0; i< new_trajectory[0].size(); i++) {
	// 	new_trajectory[1][i] = s(new_trajectory[0][i]);
	// }

	cout << "SIZE OF POINTS FOR SPLINE: " << this->points_for_spline[0].size() << endl;

	// int lane = (int) d_to_lane(end_d[0]);
	// double car_s = start_s[0];

	// vector<double> next_wp0 = map.getXY(car_s + 30, (2 + 4*lane));
	// vector<double> next_wp1 = map.getXY(car_s + 60, (2 + 4*lane));
	// vector<double> next_wp2 = map.getXY(car_s + 90, (2 + 4*lane));

	// this->points_for_spline[0].push_back(next_wp0[0]);
	// this->points_for_spline[0].push_back(next_wp1[0]);
	// this->points_for_spline[0].push_back(next_wp2[0]);

	// this->points_for_spline[1].push_back(next_wp0[1]);
	// this->points_for_spline[1].push_back(next_wp1[1]);
	// this->points_for_spline[1].push_back(next_wp2[1]);

	vector<vector<double>> new_trajectory = trajectory_generation(end_s[1]);




	
	return new_trajectory;
}



vector<vector<double>> Path_Planner::trajectory_generation(double ref_vel) {



	double ref_x = this->my_car.getX();
	double ref_y = this->my_car.getY();
	double ref_yaw = this->my_car.getYAW();

	cout << "Points: " << endl;
	for (int i=0; i < this->points_for_spline[0].size(); i++ )
	{
		
		double shift_x = this->points_for_spline[0][i] - ref_x;
		double shift_y = this->points_for_spline[1][i] - ref_y;

		this->points_for_spline[0][i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
		this->points_for_spline[1][i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));

	}

	tk::spline s;
	s.set_points(this->points_for_spline[0], this->points_for_spline[1]);

	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt(target_x*target_x + target_y*target_y);

	double x_add_on = 0;

	// fill in rest of path planner

	for (int i = 0; i < POINTS_PER_TRAJECTORY-this->prev_path_size; i++)
	{
		double N = (target_dist/(.02*ref_vel));
		double x_point = x_add_on + target_x/N;
		double y_point = s(x_point);

		x_add_on = x_point;


		double x_ref = x_point;
		double y_ref = y_point;

		// rotate back to normal
		x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
		y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));


		x_point += ref_x;
		y_point += ref_y;

		this->trajectory[0].push_back(x_point);

		this->trajectory[1].push_back(y_point);
	}

	return this->trajectory;

}


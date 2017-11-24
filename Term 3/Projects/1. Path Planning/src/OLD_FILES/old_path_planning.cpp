//#include "Eigen-3.3/Eigen/Dense"
//#include <map>
#include <math.h>
#include <vector>
#include <string>
#include <map>
#include <iostream>
#include "spline.h"
#include "vehicle.h"


using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}



// straight line in x and y co-ordinates

vector < vector<double>> simple_path (double car_x, double car_y, double car_yaw)
{

    // define next x and y position (20ms from the prev)
    // We will set the points 0.5 m apart. Since the car moves 50 times a second, a distance of 0.5m per move 
    // will create a velocity of 25 m/s. 25 m/s is close to 50 MPH.


	vector<double> next_x_vals;
	vector<double> next_y_vals;

	vector<vector<double>> output;

    double dist_inc = 0.5;
    for(int i = 0; i < 50; i++)
    {
          next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
          next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
    }

    output.push_back(next_x_vals);
    output.push_back(next_y_vals);

    return output;
}


// circular path in x and y co-ordinates (global co-ordinates)

vector <vector<double>> circular_path (vector<double> previous_path_x, vector<double> previous_path_y, double car_x, double car_y, double car_yaw)

{

	vector <vector<double>> output;

	double pos_x;
	double pos_y;
	double angle;

	vector<double> next_x_vals;
	vector<double> next_y_vals;

	int path_size = previous_path_x.size();

	for(int i = 0; i < path_size; i++)
	{
	  next_x_vals.push_back(previous_path_x[i]);
	  next_y_vals.push_back(previous_path_y[i]);
	}

	if(path_size == 0)
	{
	  pos_x = car_x;
	  pos_y = car_y;
	  angle = deg2rad(car_yaw);
	}
	else
	{
	  pos_x = previous_path_x[path_size-1];
	  pos_y = previous_path_y[path_size-1];

	  double pos_x2 = previous_path_x[path_size-2];
	  double pos_y2 = previous_path_y[path_size-2];
	  angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
	}

	double dist_inc = 0.5;
	for(int i = 0; i < 50-path_size; i++)
	{    
	  next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
	  next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
	  pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
	  pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
	}

	output.push_back(next_x_vals);
	output.push_back(next_y_vals);


	return output;

}


vector < vector<double>> straight_on_lane (double car_s, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{

	vector<double> next_x_vals;
	vector<double> next_y_vals;

	vector<vector<double>> output;

	vector<double> xy;

	// trying to go 50 miles per hour which is approx 25m/s
    double dist_inc = 0.5; // half a meter for 20ms
    for(int i = 0; i < 50; i++)
    {
    	double next_s = car_s + dist_inc*(i+1);
    	// center is in yellow lane seperating the 3 lanes going in the oppositie direction and your lane,
    	// the current line is the middle of the 3 in the right side which is 1.5 lanes from d = 0
    	// each lane is 4m
    	// so d is 1.5*4 = 6
    	double next_d = 6;

    	xy =  getXY( next_s, next_d, maps_s, maps_x, maps_y);

    	next_x_vals.push_back( xy[0]);
    	next_y_vals.push_back( xy[1]);
    }



    output.push_back(next_x_vals);
    output.push_back(next_y_vals);

    return output;
}


// lanes 0,1,2 are the right 3 lanes we use.
// initial vehicle lane
int lane = 1;

// double ref_vel = 49.5; //mph (a little under the speed limit)
double ref_vel = 0.0; // start vehicle at rest so that there is no issue with an instant acceleration/jerk
string state = "KL"; // initially car should keep straight

vector< vector<double> > splines_lane( vector<vector<double>> sensor_fusion , const vector<double> &maps_s, 
	const vector<double> &maps_x, const vector<double> &maps_y, vector<double> previous_path_x, vector<double> previous_path_y, 
	double end_path_s, double end_path_d , double car_x, double car_y, double car_yaw, double car_s, double car_d) 
{

	// previous path is whatever vector of points where used previously.
	// can be any number of points (Horizon is as long as you want)
	int prev_path_size = previous_path_x.size();

	vector<double> ptsx;
	vector<double> ptsy;

	double ref_x = car_x;
	double ref_y = car_y;
	double ref_yaw = deg2rad(car_yaw);

	double ref_s = car_s;
	double ref_d = car_d;

	double ref_x_prev;
	double ref_y_prev;

	// if prev path is almost empty, use the cars most recent localization values as the starting reference
	// other wise use the paths enpoints as a starting reference
	// this ensures smoothness from timestep to timestep
	if (prev_path_size < 2) 
	{
		// ensures the points are tangent
		// 2 points added are the prev timesteps observation and the current one
		ref_x_prev = car_x - cos(car_yaw);
		ref_y_prev = car_y - sin(car_yaw);




		ptsx.push_back(ref_x_prev);
		ptsx.push_back(car_x);

		ptsy.push_back(ref_y_prev);
		ptsy.push_back(car_y);

	}
	else 
	{
		ref_x = previous_path_x[prev_path_size -1];
		ref_y = previous_path_y[prev_path_size -1];

		ref_x_prev = previous_path_x[prev_path_size-2];
		ref_y_prev = previous_path_y[prev_path_size-2];

		// ensures the points are tangent

		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev );


		auto frenet_coords =  getFrenet(ref_x, ref_y, ref_yaw, maps_x, maps_y);

		ref_s = frenet_coords[0];
		ref_d = frenet_coords[1];


		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);


		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);

	}

	// if (prev_path_size > 0 )
	// {
	// 	car_s = end_path_s;
	// 	car_d = end_path_d;
	// }

	// Initialize ego car
	// velocity from miles/hour to meteres/second
	// auto coords =  getFrenet(ref_x, ref_y, ref_yaw, maps_x, maps_y);
	// double s_ego_vel = (coords[0] - coords_old[0])/0.02;
	// double d_ego_vel = (coords[1] - coords_old[1])/0.02;

	// cout << s_ego_vel <<endl;
	// cout << d_ego_vel << endl;

	// double vel_x1 = (ref_x - ref_x_prev)/0.02;
	// double vel_y1 = (ref_y - ref_y_prev)/0.02;



	Vehicle egoVehicle = Vehicle(car_s, s_ego_vel, 0, car_d, d_ego_vel, 0, state);




	// using sensor fusion to Determine wheter we are too close to other vehicles
	// we should slow down when we are too close
	map<int, Vehicle> observed_vehicles;


	// sensor_fusion - [ id,  x, y, vx, vy, s, d ]
	for (auto sf: sensor_fusion)
	{

		double vx = sf[3];
		double vy = sf[4];

		double vel_mag = sqrt(vx*vx + vy*vy);

		double s = sf[5];
		double d = sf[6];

		int v_id = sf[0];
		Vehicle observed_vehicle = Vehicle(s, vel_mag , 0, d, 0, 0 , "CS");


		observed_vehicles.insert(std::pair<int,Vehicle>(v_id,observed_vehicle));

	}

	// PREDICTION OF CARS FUTURE STATE SHOULD HAPPEN HERE
	map<int ,vector<Vehicle> > predictions;
	map<int, Vehicle>::iterator it = observed_vehicles.begin();
    while(it != observed_vehicles.end())
    {
        int v_id = it->first;
        vector<Vehicle> preds = it->second.generate_predictions();
        predictions[v_id] = preds;
        it++;
    }

	// BEHAVIOUR PLANNING
	// Update the state based on the predictions
    // vector<Vehicle> trajectory = egoVehicle.choose_next_state(predictions);
    // egoVehicle.realize_next_state(trajectory);







	// PATH THE VEHICLE WILL FOLOW

	// want a vector of spread out but evenly spaced x and y values
	// This way we can use them to interpolate it and get more points that would make our x, y resulting points smooth


	lane = egoVehicle.lane;
	state = egoVehicle.state;
	ref_vel = egoVehicle.s_d;

	// car_s and car_d values
	vector<double> next_wp0 = getXY(car_s + 30, (2 + 4*lane), maps_s, maps_x, maps_y);
	vector<double> next_wp1 = getXY(car_s + 60, (2 + 4*lane), maps_s, maps_x, maps_y);
	vector<double> next_wp2 = getXY(car_s + 90, (2 + 4*lane), maps_s, maps_x, maps_y);

	// 3 points on the lane (30m, 60m, and 90m ahead), we will then get more points from interpolating them

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	// at this point ptsx and ptsy have the cars 2 previous points and points in 30m, 60m, and 90m.


	// shift cars values so that the angle straight ahead is 0 degress

	for (int i=0; i < ptsx.size(); i++ )
	{
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
		ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));

	}

	tk::spline s;

	s.set_points(ptsx, ptsy);


	vector<double> next_x_vals;
	vector<double> next_y_vals;


	// having points from our previous path helps smooth the transition from 
	// one timestep to the next
	// we can keep track of a total of 50 points

	// the previous path vector is points the simulator has not yet used
	// Example: we sent the simulator 50 points, and it simulated 3 points of
	// motion, then we get 47 points back in previous path, and we can generate 3 more points


	for(int i = 0; i < prev_path_size; i++)
	{
	  next_x_vals.push_back(previous_path_x[i]);
	  next_y_vals.push_back(previous_path_y[i]);
	}

	// how do we break up our spline to get our desired velocity from point to point?
	// if we split the points so that
	// d = y distance c

	// num_of_points * 0.2 s * ref_vel (m/s) = d to calculate N

	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt(target_x*target_x + target_y*target_y);

	double x_add_on = 0;

	// fill in rest of path planner

	for (int i = 0; i <= 50-prev_path_size; i++)
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

		next_x_vals.push_back(x_point);

		next_y_vals.push_back(y_point);
	}

	vector <vector<double>> output;

	output.push_back(next_x_vals);
	output.push_back(next_y_vals);

	return output;


}






/*
For safety, a lane change path should optimize the distance away from other traffic. For comfort, a 
lane change path should also result in low acceleration and jerk. The acceleration and jerk part can be 
solved from linear equations for s and d functions. Examples of this can be found in the Trajectory Generation 
quizzes entitled, "Quintic Polynomial Solver" and "Polynomial Trajectory".

*/











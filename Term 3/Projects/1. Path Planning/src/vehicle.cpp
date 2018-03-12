#include "vehicle.h"

using namespace std;

double deg2rad(double x) { return x * pi() / 180; }

Vehicle::Vehicle() {
	this->id = MAIN_VEHCILE_ID; //this constructor is only used for the main vehicle, which we will distinguish will an id of -1
	this->x = 0;
	this->y = 0;

	this->s = 0;
  this->s_d = 0;
  this->s_dd = 0;

	this->d = 0;
  this->d_d = 0;
  this->d_dd = 0;

	this->yaw = 0;

	this->lane = LANES::CENTER;
}


Vehicle::Vehicle(double s, double d, double v){

  this->s_d = v;
  this->s = s;
  this->d = d;

  this->s_dd = 0;
  this->d_d = 0;
  this->d_dd = 0;

  if (d >= 0 && d < LANE_WIDTH) {
  	this->lane = LANES::LEFT;
  }
  else if (d >= LANE_WIDTH  &&  d < 2*LANE_WIDTH) {
  	this->lane = LANES::CENTER;
  }
  else if (d >= 2*LANE_WIDTH && d<= 3*LANE_WIDTH ) {
    this->lane = LANES::RIGHT;
  }
}


void Vehicle::set_main_vehicle_values(Map map, double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d ) {

  double pos_s, s_dot, s_ddot;
  double pos_d, d_dot, d_ddot;

  double pos_x, pos_y, pos_x2, pos_y2, angle, vel_x1, vel_y1,
         pos_x3, pos_y3, vel_x2, vel_y2, acc_x, acc_y;
  double yaw;

  int prev_path_size = previous_path_x.size();

  int subpath_size = min(OLD_POINTS_TO_KEEP, prev_path_size );
  


  // if prev path is almost empty, use the cars most recent localization values as the starting reference
  // other wise use the paths enpoints as a starting reference
  // this ensures smoothness from timestep to timestep
  if (prev_path_size < 4) 
  {
    pos_x = car_x;
    pos_y = car_y;
    angle = deg2rad(car_yaw);
    pos_s = car_s;
    pos_d = car_d;
    s_dot = car_speed;
    d_dot = 0;
    s_ddot = 0;
    d_ddot = 0;

  }
  else 
  {

    // consider current position to be last point of previous path to be kept
    pos_x = previous_path_x[subpath_size-1];
    pos_y = previous_path_y[subpath_size-1];
    pos_x2 = previous_path_x[subpath_size-2];
    pos_y2 = previous_path_y[subpath_size-2];
    angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
    vector<double> frenet = map.getFrenet(pos_x, pos_y, angle, map.getLocalWaypointsX(), map.getLocalWaypointsY(), map.getLocalWaypointsS());
    
    pos_s = frenet[0];
    pos_d = frenet[1];

    // determine dx, dy vector from set of interpoated waypoints, with pos_x,pos_y as reference point;
    // since interpolated waypoints are ~1m apart and path points tend to be <0.5m apart, these 
    // values can be reused for previous two points (and using the previous waypoint data may be
    // more accurate) to calculate vel_s (s_dot), vel_d (d_dot), acc_s (s_ddot), and acc_d (d_ddot)
    int next_interp_waypoint_index = map.NextWaypoint(pos_x, pos_y, angle, map.getLocalWaypointsX(), 
                                                  map.getLocalWaypointsY());
    double dx = map.getLocalWaypointsDX()[next_interp_waypoint_index - 1];
    double dy = map.getLocalWaypointsDY()[next_interp_waypoint_index - 1];
    // sx,sy vector is perpendicular to dx,dy
    double sx = -dy;
    double sy = dx;

    // calculate s_dot & d_dot
    vel_x1 = (pos_x - pos_x2) / DELTA_T;
    vel_y1 = (pos_y - pos_y2) / DELTA_T;
    // want projection of xy velocity vector (V) onto S (sx,sy) and D (dx,dy) vectors, and since S 
    // and D are unit vectors this is simply the dot products of V with S and V with D
    s_dot = vel_x1 * sx + vel_y1 * sy;
    d_dot = vel_x1 * dx + vel_y1 * dy;

    // have to get another point to calculate s_ddot, d_ddot from xy acceleration
    pos_x3 = previous_path_x[subpath_size-3];
    pos_y3 = previous_path_y[subpath_size-3];
    vel_x2 = (pos_x2 - pos_x3) / DELTA_T;
    vel_y2 = (pos_y2 - pos_y3) / DELTA_T;
    acc_x = (vel_x1 - vel_x2) / DELTA_T;
    acc_y = (vel_y1 - vel_y2) / DELTA_T;
    s_ddot = acc_x * sx + acc_y * sy;
    d_ddot = acc_x * dx + acc_y * dy;

  }


  this->x = pos_x;
  this->y = pos_y;

  this->s = pos_s;
  this->s_d = s_dot;
  this->s_dd = s_ddot;

  this->d = pos_d;
  this->d_d = d_dot;
  this->d_dd = d_ddot;

  this->yaw = angle;


  if (d > 0 && d < LANE_WIDTH) {
    this->lane = LANES::LEFT;
  }
  else if (this->d >= LANE_WIDTH  &&  this->d < 2*LANE_WIDTH) {
    this->lane = LANES::CENTER;
  }
  else if (this->d >= 2*LANE_WIDTH && this->d < 3*LANE_WIDTH ) {
    this->lane = LANES::RIGHT;
  }
}



vector<Vehicle> Vehicle::generatePrediction(double traj_start_time, double new_traj_size){

  // Generates a list of predicted s and d positions for dummy constant-speed vehicles
  // Because ego car trajectory is considered from end of previous path, we should also consider the 
  // trajectories of other cars starting at that time.

  vector<Vehicle> predictions;
  for( int i = 0; i < new_traj_size; i++)
  {
    double t = traj_start_time + (i * DELTA_T); // duration/POINTS_TO_TARGET
    double pred_s = this->s + this->s_d * t;
    predictions.push_back(Vehicle(pred_s, this->d, this->s_d));
  }
  return predictions;

}

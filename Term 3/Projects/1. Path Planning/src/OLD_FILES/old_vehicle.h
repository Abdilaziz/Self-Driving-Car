#ifndef VEHICLE_H
#define VEHICLE_H

#include <math.h>
#include <vector>
#include <string>
#include <map>

using namespace std;


class Vehicle {

public:

  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};

  double s;

  double s_d; // s velocity

  double s_dd; // s acceleration

  double d;

  double d_d; // d velocity

  double d_dd; // d acceleration

  string state;

  int lane;

  int preferred_buffer = 6; // perfered buffer between the velocity of the vehicle in front of us and our velocity

  double target_speed = 50/2.24;

  int goal_lane = 1;
  double goal_s = 9999999;


  /**
  * Constructor
  */
  // sensor fusion input format
  Vehicle();

  Vehicle(int lane, double s, double s_d, double s_dd, string state = "CS");

  Vehicle( double s, double s_d, double s_dd, double d, double d_d, double d_dd );

  Vehicle( double s, double s_d, double s_dd, double d, double d_d, double d_dd, string state );


  // void update_state(map<int ,vector<vector<double>>>   predctions, int prev_path_size);

  vector<Vehicle> generate_predictions(int horizon = 2);

  double position_at(int t);

  vector<string> successor_states();

  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);

  vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<double> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);  

  bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
  
  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  void realize_next_state(vector<Vehicle> trajectory);

  /**
  * Destructor
  */
  virtual ~Vehicle();





};


#endif
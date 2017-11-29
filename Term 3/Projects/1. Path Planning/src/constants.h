#ifndef CONSTS_H
#define CONSTS_H

#include <math.h>

constexpr double pi() { return M_PI; }


// number of waypoints to use for interpolation
#define WAYPOINTS_BEHIND 5
#define WAYPOINTS_AHEAD 5

#define VELOCITY_INCREMENT_LIMIT 0.125


const int MAIN_VEHCILE_ID = -1; // Distinguish the main vehicle with a unqiue id number

// The max s value before wrapping around the track back to 0
const double MAX_S = 6945.554; //entire tracks distance in m (s value)
const double SPEED_LEEWAY = 1;
const double SPEED_LIMIT = 50/2.24 - SPEED_LEEWAY;

//JMT GENERATION
const double DELTA_T = 0.02; //seconds between each point in the final path
const double POINTS_PER_TRAJECTORY = 50;
const int OLD_POINTS_TO_KEEP = 20;

const double SAFE_VEHICLE_DISTANCE = 30;
const double SAFE_LANE_CHANGE_SPACE = 20;

#define MAX_INSTANTANEOUS_ACCEL 10
#define MAX_INSTANTANEOUS_JERK 10       // m/s/s/s

#define EXPECTED_JERK_IN_ONE_SEC 2      // m/s/s
#define EXPECTED_ACC_IN_ONE_SEC 1       // m/s

const double LANE_WIDTH = 4.0;


// TRAJECTORY GENERATION CONSTS
const double POINTS_TO_TARGET = 20;
const double DT = 0.2;

#define VEHICLE_RADIUS 1.25              // meters

// cost function weights
const double COLLISION_COST_WEIGHT = 99999;
const double DISTANCE_COST_WEIGHT = 10;
const double IN_LANE_DISTANCE_COST_WEIGHT = 10000;
const double REWARD_SPEED_COST_WEIGHT = 1000;
const double NOT_MIDDLE_LANE_COST_WEIGHT = 100;

// Define Possible states

enum class STATES { START, KEEP_LANE, PREPARE_CHANGE_LEFT, PREPARE_CHANGE_RIGHT, CHANGE_LEFT, CHANGE_RIGHT};


// DEFINE POSSIBLE LANES

enum class LANES {LEFT, CENTER, RIGHT};




#endif
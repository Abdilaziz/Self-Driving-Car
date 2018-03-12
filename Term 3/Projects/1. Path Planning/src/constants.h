
#ifndef CONSTS_H
#define CONSTS_H

#include <math.h>
#include <string>

using namespace std;

#define WAYPOINTS_BEHIND 5
#define WAYPOINTS_AHEAD 5

const int MAIN_VEHCILE_ID = -1; // Distinguish the main vehicle with a unqiue id number
const double SPEED_LEEWAY = 1;
const double SPEED_LIMIT = 50/2.24 - SPEED_LEEWAY;
const double VEHICLE_RADIUS = 1.5;

#define VELOCITY_INCREMENT_LIMIT 0.125

const double SAFE_VEHICLE_DISTANCE = 30;
const double SAFE_LANE_CHANGE_SPACE = 20;

// The max s value before wrapping around the track back to 0
const double MAX_S = 6945.554;
const int GOAL_LANE = 6;
const double LANE_WIDTH = 4.0;

// FINAL TRAJECTORY CONSTS
const double DELTA_T = 0.02; //seconds between each point in the final path
const double POINTS_PER_TRAJECTORY = 50;
const int OLD_POINTS_TO_KEEP = 30; // 51


#define MAX_INSTANTANEOUS_ACCEL 10

// COSTS

const double COLLISION_COST = 99999;
const double NOT_MIDDLE_LANE_COST = 30; // 0;
const double GOAL_DISTANCE_COST = 30; // 300
const double INEFFICIENCY_COST = 800;
const double DISTANCE_COST = 10;
const double SPEED_COST = 1000;
const double INLINE_DISTANCE_COST = 3000;

// Define Possible states

enum class STATES { KEEP_LANE, PREPARE_CHANGE_LEFT, PREPARE_CHANGE_RIGHT, CHANGE_LEFT, CHANGE_RIGHT};


// DEFINE POSSIBLE LANES

enum class LANES {LEFT, CENTER, RIGHT};

// EXTRA????

constexpr double pi() { return M_PI; }



#endif
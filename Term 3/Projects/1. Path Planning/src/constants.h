#ifndef CONSTS_H
#define CONSTS_H

#include <math.h>

constexpr double pi() { return M_PI; }

const int MAIN_VEHCILE_ID = -1; // Distinguish the main vehicle with a unqiue id number

// The max s value before wrapping around the track back to 0
const double MAX_S = 6945.554; //entire tracks distance in m (s value)
const double SPEED_LEEWAY = 1;
const double SPEED_LIMIT = 50/2.24 - SPEED_LEEWAY;

//JMT GENERATION
const double DELTA_T = 0.02; //seconds between each point in a trajectory
const double POINTS_PER_TRAJECTORY = 50;
const int OLD_POINTS_TO_KEEP = 30;

const double SAFE_VEHICLE_DISTANCE = 20;
const double SAFE_LANE_CHANGE_SPACE = 20;



const double LANE_WIDTH = 4.0;


// TRAJECTORY GENERATION CONSTS
const double POINTS_TO_TARGET = 10;
const double DT = 0.2;


// Define Possible states

enum class STATES { START, KEEP_LANE, PREPARE_CHANGE_LEFT, PREPARE_CHANGE_RIGHT, CHANGE_LEFT, CHANGE_RIGHT};


// DEFINE POSSIBLE LANES

enum class LANES {LEFT, CENTER, RIGHT};




#endif
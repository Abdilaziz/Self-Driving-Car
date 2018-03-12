#ifndef COSTS_H
#define COSTS_H

#include <map>
#include <vector>
#include <string>
#include <math.h>
#include "vehicle.h"
#include "constants.h"
#include "helper.h"

using namespace std;

class Costs {

public:
    Costs(){};
    ~Costs(){};

    Helper helper;

    /*
    *   Calculates cost of a trajectory
    *
    */
    double calculate_cost(vector<vector<double>> trajectory, map<int, vector<Vehicle>> predictions, STATES state) {

        double cost = 0.0;

        vector<double> s_traj = trajectory[0];
        vector<double> d_traj = trajectory[1];

        double collision = COLLISION_COST*collision_cost(s_traj, d_traj, predictions, state);
        double N_M_L = NOT_MIDDLE_LANE_COST*not_middle_lane_cost(s_traj, d_traj, predictions, state);
        double goal = GOAL_DISTANCE_COST*goal_distance_cost(s_traj, d_traj, predictions, state);
        double ineff = INEFFICIENCY_COST*inefficiency_cost(s_traj, d_traj, predictions, state);
        double distance = DISTANCE_COST*distance_cost(s_traj, d_traj, predictions, state);
        double speed = SPEED_COST*speed_cost(s_traj, d_traj, predictions, state);
        double inline_cost = INLINE_DISTANCE_COST* in_lane_distance_cost(s_traj, d_traj, predictions, state);

        // cout << "Collision Cost: " << collision << endl;
        // cout << "Not Middle Cost: " << N_M_L << endl;
        // cout << "Distance Cost: " << distance << endl;
        // cout << "Speed Cost: " << speed << endl;
        // cout << "Inline Distance Cost: " << inline_cost << endl;
        // cout << "Goal Cost: " << goal << endl;
        // cout << "Ineff Cost: " << ineff << endl;

        cost = collision + N_M_L + distance + speed + inline_cost + goal + ineff;

        return cost;

    }

    // Binary Cost Function
    // Cost is either max for a trajectory that has a collision or 0 for a trajectory that doesnt have a collision
    double collision_cost( vector<double> s_traj, vector<double> d_traj , map<int, vector<Vehicle>> predictions, STATES state) {
        double distance = closest_vehicle_distance(s_traj, d_traj, predictions);
        // cout << "Closest Distance: " << distance << endl;
        if (distance <= 2*VEHICLE_RADIUS) {
            return 1;
        }
        else {
            return 0;
        }
    }

    double not_middle_lane_cost(vector<double> s_traj, vector<double> d_traj , map<int, vector<Vehicle>> predictions, STATES state) {

        // using intended lane so cost of prepare lane change is less when in a lane that is not the middle.
        double intended_lane = getIntendedLane(d_traj, state);
        // cout << "Intended Lane: " << intended_lane << endl;
        return logistic(pow(intended_lane - 6, 2));
    }

    double in_lane_distance_cost (vector<double> s_traj, vector<double> d_traj , map<int, vector<Vehicle>> predictions, STATES state) {
        double distance = distance_to_vehicle_in_lane(s_traj, d_traj , predictions);
        return logistic(1/distance);
    }

    double goal_distance_cost(vector<double> s_traj, vector<double> d_traj , map<int, vector<Vehicle>> predictions, STATES state) {
        double distance = getDistanceToGoal(s_traj);
        double cost;

        cost = logistic(distance);

        return cost;
    }

    double inefficiency_cost(vector<double> s_traj, vector<double> d_traj , map<int, vector<Vehicle>> predictions, STATES state) {
        /*
        Cost becomes higher for trajectories with intended lane and final lane that have slower traffic. 
        */

        double proposed_speed_intended = lane_speed(s_traj, predictions, helper.d_to_lane(getIntendedLane(d_traj, state)));
        if (proposed_speed_intended < 0) {
            proposed_speed_intended = SPEED_LIMIT;
        }

        double proposed_speed_final = lane_speed(s_traj, predictions, helper.d_to_lane(getFinalLane(d_traj)));
        if (proposed_speed_final < 0) {
            proposed_speed_final = SPEED_LIMIT;
        }
        
        double cost = (2.0*SPEED_LIMIT - proposed_speed_intended - proposed_speed_final)/SPEED_LIMIT;

        return logistic(cost);
    }

    double distance_cost(vector<double> s_traj, vector<double> d_traj , map<int, vector<Vehicle>> predictions, STATES state) {
        double distance = closest_vehicle_distance(s_traj, d_traj, predictions);
        return logistic(2*VEHICLE_RADIUS/distance);
    }

    //reward trajectories with higher speeds
    double speed_cost(vector<double> s_traj, vector<double> d_traj , map<int, vector<Vehicle>> predictions, STATES state) {
        double size = s_traj.size();
        double last_speed = (s_traj[size-1] - s_traj[size-2])/DELTA_T; //DT
        return logistic((SPEED_LIMIT - last_speed)/ SPEED_LIMIT);

    }

    double closest_vehicle_distance(vector<double> s_traj, vector<double> d_traj, map<int, vector<Vehicle>> predictions ) {
        double min_distance = MAX_S;

        for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
            double distance = distance_to_vehicle(s_traj, d_traj, it->second);
            if (distance < min_distance) {
                min_distance = distance;
            }
        }
        return min_distance;
    }

    double distance_to_vehicle_in_lane(vector<double> s_traj, vector<double> d_traj, map<int, vector<Vehicle>> predictions) {
        double min_distance = MAX_S;
        double traj_size = s_traj.size();

        for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
            
            if (it->second[traj_size-1].getLane() == helper.d_to_lane(d_traj[traj_size-1]) ) {
                double distance = distance_to_vehicle(s_traj, d_traj, it->second);
                if (distance < min_distance) {
                    min_distance = distance;
                }
            }

        }
        return min_distance;
    }

    double distance_to_vehicle(vector<double> s_traj, vector<double> d_traj, vector<Vehicle> prediction) {
        double closest_distance = MAX_S;
        double new_points = s_traj.size();

        for (int i = 0; i < new_points; i++) {
            double distance = sqrt(pow(s_traj[i] - prediction[i].getS(), 2) + pow(d_traj[i] - prediction[i].getD(), 2));
            if (distance < closest_distance) {
                closest_distance = distance;
            }
        }
        return closest_distance;
    }

    // determine the average speed of a lane
    double lane_speed(vector<double> s_traj, map<int, vector<Vehicle>> predictions, LANES lane) {

        double avg_lane_speed = 0.0;
        int numb = 0;
        for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
            Vehicle vehicle = it->second[0]; // vehicle predictions are made with a constant velocity model
            double distance = abs(vehicle.getS() - s_traj[0]);
            if (vehicle.getLane() == lane && distance < SAFE_VEHICLE_DISTANCE  ) {
                // speed of a car in the same lane
                avg_lane_speed = vehicle.getS_D();
                numb += 1;
            }
        }

        if ( numb == 0) {
            return -1;
        } else {
            return (avg_lane_speed/numb);
        }

        
    }


    double getDistanceToGoal(vector<double> s_traj) {
        int traj_size = s_traj.size();
        double final_s = s_traj[traj_size-1];

        double distance_to_goal = MAX_S - final_s;

        return distance_to_goal;
    }

    double getFinalLane(vector<double> d_traj) {

        int traj_size = d_traj.size();
        double final_d = d_traj[traj_size-1];

        //ANES final_lane = d_to_lane(final_d);

        return final_d;
    }


    double getIntendedLane( vector<double> d_traj , STATES state) {

        // int traj_size = d_traj.size();

        double initial_d = d_traj[0];

        LANES initial_lane = helper.d_to_lane(initial_d);

        LANES intended_lane = helper.changeLane(state, initial_lane);

        return helper.lane_to_d(intended_lane);
    }

    double logistic(double x) {
        // Return a value between -1 and 1
        return 2.0 / (1 + exp(-x)) - 1.0;
    }
};


#endif
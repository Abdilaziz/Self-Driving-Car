#ifndef COSTS
#define COSTS


#include <vector>
#include <algorithm>
#include <math.h>
#include "constants.h"

using namespace std;

class Costs {

public:
  Costs(){};
  ~Costs(){};

  double logistic(double x){
    // Return a value between -1 and 1
    return 2.0 / (1 + exp(-x)) - 1.0;
  }

  double distance_to_vehicle(vector<double> s_traj, vector<double> d_traj, vector<vector<double>> prediction) {
    double min_distance = 99999;
    for (int i = 0; i < POINTS_TO_TARGET; i++) {
      double current_dist = sqrt(pow(s_traj[i] - prediction[i][0], 2) + pow(d_traj[i] - prediction[i][1], 2));

      if (current_dist < min_distance) {
        min_distance = current_dist;
      }
    }
    return min_distance;
  }

  double distance_to_closest_vehicle(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
    double min_distance = 99999;
    for (auto prediction : predictions) {
      double current_dist = distance_to_vehicle(s_traj, d_traj, prediction.second);
      if (current_dist < min_distance) {
        min_distance = current_dist;
      }
    }
    return min_distance;
  }

  double distance_to_closest_vehicle_in_lane(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
    double min_distance = 99999;
    int cur_lane = (d_traj[d_traj.size() - 1] / 4);
    for (auto prediction : predictions) {
      vector<vector<double>> pred_traj = prediction.second;
      int pred_lane = (pred_traj[pred_traj.size() - 1][1] / 4);
      if (cur_lane == pred_lane) {
        double current_dist = distance_to_vehicle(s_traj, d_traj, prediction.second);
        if (current_dist < min_distance && current_dist < 30) {
          min_distance = current_dist;
        }
      }
    }
    return min_distance;
  }

  vector<double> differentiate_trajectory(vector<double> traj) {
    vector<double> diff;
    for (int i = 1; i < traj.size(); i++) {
      diff.push_back((traj[i] - traj[i-1]) / DT);
    }
    return diff;
  }


  // COST FUNCTIONS

  // Binary cost function
  double collision_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
    double distance = distance_to_closest_vehicle(s_traj, d_traj, predictions);
    if (distance < 2 * VEHICLE_RADIUS) 
    {
      return 1;
    } 
    else 
    { 
      return 0;
    }
  }

  double distance_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
    double distance = distance_to_closest_vehicle(s_traj, d_traj, predictions);
    return logistic(2 * VEHICLE_RADIUS / distance);
  }

  double in_lane_distance_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
    double distance = distance_to_closest_vehicle_in_lane(s_traj, d_traj, predictions);
    return logistic(1/distance);
  }

  double reward_speed_cost(vector<double> s_traj) {
    vector<double> s_dot_traj = differentiate_trajectory(s_traj);
    double final_s_dot, total = 0;

    final_s_dot = s_dot_traj[s_dot_traj.size() - 1];
    return logistic((SPEED_LIMIT - final_s_dot) / SPEED_LIMIT);
  }

  double not_middle_lane_cost(vector<double> d_traj) {
    double end_d = d_traj[d_traj.size()-1];
    return logistic(pow(end_d-6, 2));
  }

  double calculate_total_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {

    double total_cost = 0;
    double collision = collision_cost(s_traj, d_traj, predictions) * COLLISION_COST_WEIGHT;
    double distance = distance_cost(s_traj, d_traj, predictions) * DISTANCE_COST_WEIGHT;
    double in_lane_distance = in_lane_distance_cost(s_traj, d_traj, predictions) * IN_LANE_DISTANCE_COST_WEIGHT;
    double reward_speed = reward_speed_cost(s_traj) * REWARD_SPEED_COST_WEIGHT;
    double not_middle_lane = not_middle_lane_cost(d_traj) * NOT_MIDDLE_LANE_COST_WEIGHT;


    total_cost += collision + distance + in_lane_distance + reward_speed + not_middle_lane;

    return total_cost;
  }

};

#endif
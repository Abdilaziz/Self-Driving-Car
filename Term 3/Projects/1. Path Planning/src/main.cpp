#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include <math.h>
#include "map.h"
#include "vehicle.h"
#include "path_planning.h"
#include "road.h"
#include "prediction.h"
#include "constants.h"


using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double deg2rad(double x) { return x * pi() / 180; }


int main() {
  uWS::Hub h;


  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  Map map = Map(map_file_);
  Road road;
  Vehicle my_car;
  Path_Planner path_planner;
  Prediction prediction;

  h.onMessage([&map, &road, &my_car, &path_planner, &prediction](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

            json msgJson;

            // 50 points in each vector

            vector<double> next_x_vals;
            vector<double> next_y_vals;

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            // format for sensor_fusion [id, x, y, vx, vy, s, d]
            // use vx and vy to determine where the cars will be in the future
            
            // For instance, if you were to assume that the tracked car kept moving along the road,
            // then its future predicted Frenet s value will be its current s value plus its (transformed)
            // total velocity (m/s) multiplied by the time elapsed into the future (s).


            // USE PREVIOUS PATH IF AVAIBLE TO SMOOTHEN THE TRAJECTORY BETWEEN TIMESTEPS
            int prev_path_size = previous_path_x.size();

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            double ref_speed = car_speed;

            double ref_s = car_s;
            double ref_d = car_d;

            double ref_x_prev;
            double ref_y_prev;

            vector<double> ptsx;
            vector<double> ptsy;

            // if prev path is almost empty, use the cars most recent localization values as the starting reference
            // other wise use the paths enpoints as a starting reference
            // this ensures smoothness from timestep to timestep
            if (prev_path_size < 2) 
            {
              // ensures the points are tangent
              // 2 points added are the prev timesteps observation and the current one
              ref_x_prev = car_x - cos(car_yaw);
              ref_y_prev = car_y - sin(car_yaw);

            }
            else 
            {
              ref_x = previous_path_x[prev_path_size -1];
              ref_y = previous_path_y[prev_path_size -1];

              ref_x_prev = previous_path_x[prev_path_size-2];
              ref_y_prev = previous_path_y[prev_path_size-2];

              // ensures the points are tangent

              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev );

              auto old_frenet_coords = map.getFrenet(ref_x_prev, ref_y_prev, ref_yaw);
              auto frenet_coords =  map.getFrenet(ref_x, ref_y, ref_yaw);

              ref_s = frenet_coords[0];
              ref_d = frenet_coords[1];


              ref_speed = (ref_s - old_frenet_coords[0])/DELTA_T;


            }

            std::cout << "REF D RECIEVED: " << ref_d << endl;

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);


            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

            vector<vector<double>> points_for_spline = {ptsx, ptsy};


            my_car.set_main_vehicle_values(ref_x, ref_y, car_speed, ref_s, ref_d, ref_yaw);
            //my_car.set_main_vehicle_values(car_x, car_y, car_speed, car_s, car_d, car_yaw);


            vector<Vehicle> left_lane;
            vector<Vehicle> center_lane;
            vector<Vehicle> right_lane;

            // organize observed vehicles into seperate vectors for what lane they are currently in
            for (int i = 0; i < sensor_fusion.size(); i++) {

              int id = sensor_fusion[i][0];
              double x = sensor_fusion[i][1];
              double y = sensor_fusion[i][2];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double s = sensor_fusion[i][5];
              double d = sensor_fusion[i][6];
              double v = sqrt(vx*vx + vy*vy);

              Vehicle obsv_vehicle = Vehicle(id, x, y, v, s, d);
              LANES lane = obsv_vehicle.getLane();

              if (lane == LANES::LEFT) {
                left_lane.push_back(obsv_vehicle);
              } else if (lane == LANES::CENTER) {
                center_lane.push_back(obsv_vehicle);
              } else {
                right_lane.push_back(obsv_vehicle);
              }
            }

            
            // previous_path that is returned from the simulator is the points from
            // the previously sent trajectory that have yet to be used

            // each timestep we only add points to the end of the path so that
            // the trajectory always sent is the same number of points (50)

            // for (int i = 0; i < prev_path_size; i++ ) {
            //   next_x_vals.push_back(previous_path_x[i]);
            //   next_y_vals.push_back(previous_path_y[i]);
            // }

            road.setVehiclesOnRoad(left_lane, center_lane, right_lane);

            // vector<vector<double>> current_trajectory = {next_x_vals, next_y_vals};


            vector<vector<double>> current_trajectory = {previous_path_x, previous_path_y};



            vector<vector<double>> new_trajectory = path_planner.get_new_trajectory(map, road, my_car, current_trajectory, prev_path_size, points_for_spline);

            // straight path
            //vector<vector<double>> examplePath;
            //examplePath =  simple_path(car_x, car_y, car_yaw);
            //examplePath = circular_path (previous_path_x, previous_path_y, car_x, car_y, car_yaw);
            //examplePath = straight_on_lane( car_s, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            //examplePath = splines_lane( sensor_fusion , map_waypoints_s, map_waypoints_x, map_waypoints_y, previous_path_x, previous_path_y, end_path_s, end_path_d ,  car_x,  car_y,  car_yaw,  car_s, car_d);

            //next_x_vals = examplePath[0];
            //next_y_vals = examplePath[1];

            next_x_vals = new_trajectory[0];
            next_y_vals = new_trajectory[1];

            std::cout << "sent trajectory to sim. Size: " << next_x_vals.size() << endl; 
            std::cout << " " << endl;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

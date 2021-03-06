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

#include "constants.h"
#include "map.h"
#include "vehicle.h"
#include "road.h"
#include "path_planner.h"

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


int main() {
  uWS::Hub h;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // initialize the waypoint data
	Map world = Map(map_file_);
  Vehicle my_car;
  Road road;
  Path_Planner path_planner;

  h.onMessage([&world, &my_car, &road, &path_planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            // uses the previous 5 and next 5 waypoints to interpolate waypoints to more accurate convert between Frenet Coordinates and Cartesian Coordinates
						world.update_local_waypoints(car_x, car_y, car_yaw);

            my_car.set_main_vehicle_values( world, car_x,  car_y,  car_s,  car_d,  car_yaw, car_speed, 
                                            previous_path_x, previous_path_y,  end_path_s,  end_path_d );

            int prev_path_size = previous_path_x.size();
            int subpath_size = min(OLD_POINTS_TO_KEEP, prev_path_size);
            int new_traj_size = POINTS_PER_TRAJECTORY - subpath_size;

            double traj_start_time = subpath_size * DELTA_T; // Simulator used 50 points/s so each point is 0.02s of the path

            // cout << "NEW POINTS: " << new_traj_size << endl;

            map<int, vector<Vehicle>> predictions;

            // generate predictions for vehicles in the horizon (horizon is new_traj_size*DELTA_T)

            for (int i = 0; i < sensor_fusion.size(); i++) {

              int id = sensor_fusion[i][0];
              double x = sensor_fusion[i][1];
              double y = sensor_fusion[i][2];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double s = sensor_fusion[i][5];
              double d = sensor_fusion[i][6];
              double v = sqrt(vx*vx + vy*vy);

              Vehicle obsv_vehicle = Vehicle(s, d, v);

              vector<Vehicle> prediction = obsv_vehicle.generatePrediction(traj_start_time, new_traj_size);

              predictions[id] = prediction;

            }

            road.setPredictions(predictions);

            vector<vector<double>> current_trajectory = {previous_path_x, previous_path_y};


            // send all data to path planner to determine a new optimal trajectory that includes our previous path x and y
            vector<vector<double>> new_trajectory = path_planner.get_new_trajectory(world, road, my_car, current_trajectory, subpath_size);


            next_x_vals = new_trajectory[0];
            next_y_vals = new_trajectory[1];

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

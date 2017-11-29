
#include "map.h"

using namespace std;
using namespace tk;



vector<double> Map::interpolate_points(vector<double> pts_x, vector<double> pts_y, double interval, int output_size) {
  // uses the spline library to interpolate points connecting a series of x and y values
  // output is output_size number of y values beginning at y[0] with specified fixed interval

  if (pts_x.size() != pts_y.size()) {
    cout << "ERROR! SMOOTHER: interpolate_points size mismatch between pts_x and pts_y" << endl;
    return { 0 };
  }

  tk::spline s;
  s.set_points(pts_x,pts_y);    // currently it is required that X is already sorted
  vector<double> output;
  for (int i = 0; i < output_size; i++) {
    output.push_back(s(pts_x[0] + i * interval));
  }
  return output;
}

vector<double> Map::interpolate_points(vector<double> pts_x, vector<double> pts_y, 
                                  vector<double> eval_at_x) {
  // uses the spline library to interpolate points connecting a series of x and y values
  // output is spline evaluated at each eval_at_x point

  if (pts_x.size() != pts_y.size()) {
    cout << "ERROR! SMOOTHER: interpolate_points size mismatch between pts_x and pts_y" << endl;
    return { 0 };
  }

  tk::spline s;
  s.set_points(pts_x,pts_y);    // currently it is required that X is already sorted
  vector<double> output;
  for (double x: eval_at_x) {
    output.push_back(s(x));
  }
  return output;
}

Map::Map(string map_file) {

  // Load up map values for waypoint's x,y,s and d normalized normal vectors

  // map data is a list of waypoints 
  ifstream in_map_(map_file.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    this->map_waypoints_x.push_back(x);
    this->map_waypoints_y.push_back(y);
    this->map_waypoints_s.push_back(s);
    this->map_waypoints_dx.push_back(d_x);
    this->map_waypoints_dy.push_back(d_y);
  }


}


double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int Map::ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int Map::NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Map::getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y, vector<double> maps_s)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }
  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];
  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;
  double frenet_d = distance(x_x,x_y,proj_x,proj_y);
  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);
  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }
  // calculate s value
  double frenet_s = maps_s[0];
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }
  frenet_s += distance(0,0,proj_x,proj_y);
  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Map::getXY(double s, double d )
{
  int prev_wp = -1;
  while(s > this->interpolated_local_waypoints_s[prev_wp+1] && (prev_wp < (int)(this->interpolated_local_waypoints_s.size()-1) ))
  {
    prev_wp++;
  }
  int wp2 = (prev_wp+1)%this->interpolated_local_waypoints_x.size();
  double heading = atan2((this->interpolated_local_waypoints_y[wp2]-this->interpolated_local_waypoints_y[prev_wp]),(this->interpolated_local_waypoints_x[wp2]-this->interpolated_local_waypoints_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-this->interpolated_local_waypoints_s[prev_wp]);
  double seg_x = this->interpolated_local_waypoints_x[prev_wp]+seg_s*cos(heading);
  double seg_y = this->interpolated_local_waypoints_y[prev_wp]+seg_s*sin(heading);
  double perp_heading = heading-pi()/2;
  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);
  return {x,y};
}


void Map::update_local_waypoints(double car_x, double car_y, double car_yaw) {

  int num_waypoints = this->map_waypoints_x.size();
  int next_waypoint_index = NextWaypoint(car_x, car_y, car_yaw, this->map_waypoints_x, this->map_waypoints_y);

  vector<double> coarse_waypoints_s, coarse_waypoints_x, coarse_waypoints_y, 
                 coarse_waypoints_dx, coarse_waypoints_dy;


  for (int i = -WAYPOINTS_BEHIND; i < WAYPOINTS_AHEAD; i++) {

    int idx = (next_waypoint_index+i) % num_waypoints;
    if (idx < 0) {
      idx += num_waypoints;
    }
    double current_s = map_waypoints_s[idx];
    double base_s = map_waypoints_s[next_waypoint_index];
    if (i < 0 && current_s > base_s) {
      current_s -= MAX_S;
    }
    if (i > 0 && current_s < base_s) {
      current_s += MAX_S;
    }
    coarse_waypoints_s.push_back(current_s);
    coarse_waypoints_x.push_back(map_waypoints_x[idx]);
    coarse_waypoints_y.push_back(map_waypoints_y[idx]);
    coarse_waypoints_dx.push_back(map_waypoints_dx[idx]);
    coarse_waypoints_dy.push_back(map_waypoints_dy[idx]);
  }

  double dist_inc = 0.5;  
  int num_interpolation_points = (coarse_waypoints_s[coarse_waypoints_s.size()-1] - coarse_waypoints_s[0]) / dist_inc;

  this->interpolated_local_waypoints_s = {};
  this->interpolated_local_waypoints_x = {};
  this->interpolated_local_waypoints_y = {};
  this->interpolated_local_waypoints_dx = {};
  this->interpolated_local_waypoints_dy = {};

  this->interpolated_local_waypoints_s.push_back(coarse_waypoints_s[0]);
  for (int i = 1; i < num_interpolation_points; i++) {
    this->interpolated_local_waypoints_s.push_back(coarse_waypoints_s[0] + i * dist_inc);
  }
  this->interpolated_local_waypoints_x = interpolate_points(coarse_waypoints_s, coarse_waypoints_x, dist_inc, num_interpolation_points);
  this->interpolated_local_waypoints_y = interpolate_points(coarse_waypoints_s, coarse_waypoints_y, dist_inc, num_interpolation_points);
  this->interpolated_local_waypoints_dx = interpolate_points(coarse_waypoints_s, coarse_waypoints_dx, dist_inc, num_interpolation_points);
  this->interpolated_local_waypoints_dy = interpolate_points(coarse_waypoints_s, coarse_waypoints_dy, dist_inc, num_interpolation_points);
}



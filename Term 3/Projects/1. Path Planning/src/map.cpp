
#include "map.h"

using namespace std;
using namespace tk;

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


Map::Map(string map_file) {

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }


  // create a spline from the points read from the file
  // maps s points to each x, y, dx, and dy
  // this will lead to an easy conversion from frenet coords to xy coords

  this->wp_x_spline.set_points(map_waypoints_s, map_waypoints_x);
  this->wp_y_spline.set_points(map_waypoints_s, map_waypoints_y);
  this->wp_dx_spline.set_points(map_waypoints_s, map_waypoints_dx);
  this->wp_dy_spline.set_points(map_waypoints_s, map_waypoints_dy);

  this->maps_x = map_waypoints_x;
  this->maps_y = map_waypoints_y;
  this->maps_s = map_waypoints_s;

}



// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Map::getFrenet(double x, double y, double theta)
{
	int next_wp = this->NextWaypoint(x,y, theta);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = this->maps_x.size()-1;
	}

	double n_x = this->maps_x[next_wp]-this->maps_x[prev_wp];
	double n_y = this->maps_y[next_wp]-this->maps_y[prev_wp];
	double x_x = x - this->maps_x[prev_wp];
	double x_y = y - this->maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-this->maps_x[prev_wp];
	double center_y = 2000-this->maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(this->maps_x[i],this->maps_y[i],this->maps_x[i+1],this->maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Map::getXY(double s, double d)
{
	double wp_x, wp_y, wp_dx, wp_dy, x, y;

	wp_x = this->wp_x_spline(s);
	wp_y = this->wp_y_spline(s);
	wp_dx = this->wp_dx_spline(s);
	wp_dy = this->wp_dy_spline(s);

	x = wp_x + wp_dx*d;
	y = wp_y + wp_dy*d;

	return {x,y};

}

// vector<double> Map::getXY(double s, double d)
// {
//   int prev_wp = -1;
//   while(s > this->maps_s[prev_wp+1] && (prev_wp < (int)(this->maps_s.size()-1) ))
//   {
//     prev_wp++;
//   }
//   int wp2 = (prev_wp+1)%this->maps_x.size();
//   double heading = atan2((this->maps_y[wp2]-this->maps_y[prev_wp]),(this->maps_x[wp2]-this->maps_x[prev_wp]));
//   // the x,y,s along the segment
//   double seg_s = (s-this->maps_s[prev_wp]);
//   double seg_x = this->maps_x[prev_wp]+seg_s*cos(heading);
//   double seg_y = this->maps_y[prev_wp]+seg_s*sin(heading);
//   double perp_heading = heading-pi()/2;
//   double x = seg_x + d*cos(perp_heading);
//   double y = seg_y + d*sin(perp_heading);
//   return {x,y};
// }



int Map::ClosestWaypoint(double x, double y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < this->maps_x.size(); i++)
  {
    double map_x = this->maps_x[i];
    double map_y = this->maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int Map::NextWaypoint(double x, double y, double theta)
{

  int closestWaypoint = this->ClosestWaypoint(x,y);

  double map_x = this->maps_x[closestWaypoint];
  double map_y = this->maps_y[closestWaypoint];

  double heading = atan2( (map_y-y),(map_x-x) );

  double angle = abs(theta-heading);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;

}

#include "vehicle.h"

using namespace std;

Vehicle::Vehicle() {
	this->id = MAIN_VEHCILE_ID; //this constructor is only used for the main vehicle, which we will distinguish will an id of -1
	this->x = 0;
	this->y = 0;
	this->v = 0;
	this->s = 0;
	this->d = 0;
	this->yaw = 0;

	this->lane = LANES::CENTER;
}


Vehicle::Vehicle(int id, double x, double y, double v, double s, double d){
  this->id = id;
  this->x = x;
  this->y = y;
  this->v = v;
  this->s = s;
  this->d = d;

  if (d > 0 && d < LANE_WIDTH) {
  	this->lane = LANES::LEFT;
  }
  else if (this->d >= LANE_WIDTH  &&  this->d < 2*LANE_WIDTH) {
  	this->lane = LANES::CENTER;
  }
  else if (this->d >= 2*LANE_WIDTH && this->d <= 3*LANE_WIDTH ) {
    this->lane = LANES::RIGHT;
  }
}


void Vehicle::set_main_vehicle_values(double x, double y, double v, double s, double d, double yaw) {

  this->x = x;
  this->y = y;
  this->v = v;
  this->s = s;
  this->d = d;
  this->yaw = yaw;


  if (d > 0 && d < LANE_WIDTH) {
    this->lane = LANES::LEFT;
  }
  else if (this->d >= LANE_WIDTH  &&  this->d < 2*LANE_WIDTH) {
    this->lane = LANES::CENTER;
  }
  else if (this->d >= 2*LANE_WIDTH && this->d <= 3*LANE_WIDTH ) {
    this->lane = LANES::RIGHT;
  }
  
}

int Vehicle::getID(){return this->id;};
double Vehicle::getX(){return this->x;};
double Vehicle::getY(){return this->y;};
double Vehicle::getV(){return this->v;};
double Vehicle::getS(){return this->s;};
double Vehicle::getD(){return this->d;};
double Vehicle::getYAW(){return this->yaw;};

LANES Vehicle::getLane(){return this->lane;};

vector<double> Vehicle::getPREV_S() { return this->previous_s;}
vector<double> Vehicle::getPREV_D() { return this->previous_d;}

void Vehicle::setPREV_S(vector<double> previous_s) { this->previous_s = previous_s;}
void Vehicle::setPREV_D(vector<double> previous_d) { this->previous_d = previous_d;}




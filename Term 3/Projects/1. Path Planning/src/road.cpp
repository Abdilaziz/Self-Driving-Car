#include "road.h"



void Road::setVehiclesOnRoad(vector<Vehicle> left_lane, vector<Vehicle> center_lane, vector<Vehicle> right_lane) {
	this->left_lane = left_lane;
	this->center_lane = center_lane;
	this->right_lane = right_lane;
}


// checking if there is ample distance between my_car and the vehicle ahead in the same lane (dist)
bool Road::isVehicleAhead(Vehicle &my_car, LANES lane) {
	vector<Vehicle> vehicles;
	if (lane == LANES::LEFT) {
		vehicles = this->left_lane;
	}
	else if (lane == LANES::CENTER) {
		vehicles = this->center_lane;
	}
	else {
		vehicles = this->right_lane;
	}

	bool vehicleAhead = false;

	for (int i = 0; i < vehicles.size(); i++) {
		double distance = vehicles[i].getS() - my_car.getS();
		if ( (distance > 0) && (distance < SAFE_VEHICLE_DISTANCE) ) {
			this->vehicle_ahead = vehicles[i];
			vehicleAhead = true;
			break;
		}
	}

	return vehicleAhead;
}

bool Road::isLaneChangeSafe(Vehicle &my_car, LANES lane) {
	vector<Vehicle> vehicles;
	if (lane == LANES::LEFT) {
		vehicles = this->left_lane;
	}
	else if (lane == LANES::CENTER) {
		vehicles = this->center_lane;
	}
	else {
		vehicles = this->right_lane;
	}

	bool laneIsSafe = true;


	for (int i = 0; i < vehicles.size(); i++) {
		double distance = fabs(vehicles[i].getS() - my_car.getS());
		if ( distance < SAFE_LANE_CHANGE_SPACE) {
			laneIsSafe = false;
			break;
		}
	}

	return laneIsSafe;
}


LANES Road::availableLane(Vehicle &my_car) {

  LANES car_lane = my_car.getLane();
  LANES target_lane = car_lane;

  if (car_lane == LANES::LEFT || car_lane == LANES::RIGHT) {
    if (this->isLaneChangeSafe(my_car, LANES::CENTER)) {
      target_lane = LANES::CENTER;
    }
  } else {
    if (this->isLaneChangeSafe(my_car, LANES::LEFT)) {
      target_lane = LANES::LEFT;
    } else if (this->isLaneChangeSafe(my_car, LANES::RIGHT)) {
      target_lane = LANES::RIGHT;
    }
  }
  return target_lane;

}

bool Road::isLaneChangeSafe(Vehicle &my_car, int lane) {
	LANES cur_lane = (LANES) lane;
	return isLaneChangeSafe(my_car, cur_lane);
}

bool Road::isVehicleAhead(Vehicle &my_car, int lane) {
	LANES cur_lane = (LANES) lane;
	return isVehicleAhead(my_car, cur_lane);
}

#include "road.h"


bool Road::isVehicleAhead(double s, LANES lane) {

	double min_s = MAX_S;
	bool found_vehicle = false;
    Vehicle temp_vehicle;

    for (map<int, vector<Vehicle>>::iterator it = this->predictions.begin(); it != this->predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        double distance = temp_vehicle.getS() - s;
        if (temp_vehicle.getLane() == lane && distance >= 0 && distance <= SAFE_VEHICLE_DISTANCE && temp_vehicle.getS() < min_s ) { // && ((temp_vehicle.getS() - s) < SAFE_VEHICLE_DISTANCE ) 
            min_s = temp_vehicle.getS();
            this->vehicle_ahead = temp_vehicle;
            found_vehicle = true;
        }
	}
    // cout << "Closest Ahead Distance: " <<  (min_s-s) << endl;
	return found_vehicle;
}


bool Road::isVehicleBehind(double s, LANES lane) {

	double max_s = -1;
	bool found_vehicle = false;
    Vehicle temp_vehicle;

    for (map<int, vector<Vehicle>>::iterator it = this->predictions.begin(); it != this->predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        double distance = s - temp_vehicle.getS();
        if (temp_vehicle.getLane() == lane && distance >= 0 && distance <= SAFE_VEHICLE_DISTANCE && temp_vehicle.getS() > max_s ) { // && ((s-temp_vehicle.getS()) < SAFE_VEHICLE_DISTANCE ) 
            max_s = temp_vehicle.getS();
            this->vehicle_behind = temp_vehicle;
            found_vehicle = true;
        }
	}

    // cout << "Closest Behind Distance: " << (s-max_s) << endl;

	return found_vehicle;
}
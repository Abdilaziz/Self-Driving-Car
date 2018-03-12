#ifndef HELPER_H
#define HELPER_H


#include <string>
#include "constants.h"

using namespace std;

class Helper {

public:
    Helper(){};
    ~Helper(){};

    string lane_to_string(LANES lane) {
        if (lane == LANES::LEFT){
            return "LEFT";
        } else if(lane == LANES::CENTER){
            return "CENTER";
        } else{
            return "RIGHT";
        }
    }

    string state_to_string(STATES state){
        if (state == STATES::KEEP_LANE){
            return "KEEP_LANE";
        } 
        else if (state == STATES::PREPARE_CHANGE_LEFT ) {
            return "PREPARE_CHANGE_LEFT";
        }
        else if (state == STATES::PREPARE_CHANGE_RIGHT ){
            return "PREPARE_CHANGE_RIGHT";
        }
        else if(state == STATES::CHANGE_LEFT){
            return "CHANGE_LEFT";
        }
        else{
            return "CHANGE_RIGHT";
        }
    }

    LANES changeLane(STATES state, LANES lane) {
        LANES newLane;
        if (state == STATES::PREPARE_CHANGE_LEFT || state == STATES::CHANGE_LEFT) {
            if (lane == LANES::CENTER) {
                newLane = LANES::LEFT;
            }
            else if (lane == LANES::RIGHT) {
                newLane = LANES::CENTER;
            }
        }
        else if (state == STATES::PREPARE_CHANGE_RIGHT || state == STATES::CHANGE_RIGHT) {
            if (lane == LANES::CENTER) {
                newLane = LANES::RIGHT;
            }
            else if (lane == LANES::LEFT) {
                newLane = LANES::CENTER;
            }
        }
        else {
            newLane = lane;
        }

        return newLane;
    }

    LANES d_to_lane(double d) {
        LANES lane;
        if (d > 0 && d < LANE_WIDTH) {
            lane = LANES::LEFT;
        }
        else if (d >= LANE_WIDTH  &&  d < 2*LANE_WIDTH) {
            lane = LANES::CENTER;
        }
        else if (d >= 2*LANE_WIDTH && d <= 3*LANE_WIDTH ) {
            lane = LANES::RIGHT;
        }
        return lane;
    }

    double lane_to_d(LANES lane) {
        return ((double) lane)*LANE_WIDTH + 2.0;
    }


};




#endif

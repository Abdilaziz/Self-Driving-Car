//============================================================================
// Name        : bayesianFilter.cpp
// Version     : 1.0.0
// Copyright   : MBRDNA, Udacity
//============================================================================

#include "bayesianFilter.h"
#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;

//constructor:
bayesianFilter::bayesianFilter() {

    //TODO add is_initialized to header file
    //set initialization to false:
    // NOTE: helps us set up the initial believe state
	is_initialized_ = false;

    //TODO add control_std to header file
	//set standard deviation of control to 1.0f:
	control_std = 1.0f;

	//define size of believe, same size as map
	bel_x.resize(100,0);
	
	//TODO add bel_x_init to header file
	// NOTE: helps us not overwrite believe during
	// the motion calculation
	bel_x_init.resize(100,0); // because map is from 0-99, position estimate is the probability values in a vector the size of 100

}

//de-constructor:
bayesianFilter::~bayesianFilter() {

}

void bayesianFilter::process_measurement(const MeasurementPackage &measurements,
        						             const map &map_1d,
                                         help_functions &helpers){

	/******************************************************************************
	 *  Set init belief of state vector:
	 ******************************************************************************/
	 //if(!is_initialized_){

		//TODO: run over map, all map_1d.lanmark_list values:
		//for (int l=0; l< ...){

			//TODO: get landmark l from map 
			

			//check, if landmark position x fits in map [0,100]:
			//if(... ){

				//TODO: get landmark x position * use help_function.h for reference
				
				// ______/---\_____ << initial believe state at a landmark
				
				//TODO: set belief to 1 at position and +/- from position:
				

		//	} //end if
		//}//end for

	//TODO: normalize initial believe * use help_function.h for reference
	//bel_x_init = 

	//set initial flag to true:
	//is_initialized_ = 
	
	//}//end if

	if (!is_initialized_){

		for (int i = 0; i<map_1d.landmark_list.size(); i++){

			map::single_landmark_s landmark_temp;

			landmark_temp = map_1d.landmark_list[i];

			if (landmark_temp.x_f > 0 && landmark_temp.x_f < bel_x_init.size()) {

				int position_x = int(landmark_temp.x_f);

				bel_x_init[position_x] = 1.0f;

				bel_x_init[position_x - 1] = 1.0f;
				bel_x_init[position_x + 1] = 1.0f;

			}

		}

		bel_x_init = helpers.normalize_vector(bel_x_init);

		is_initialized_ = true;

	}



	
	/******************************************************************************
	 *  motion model and observation update
	******************************************************************************/
	std::cout <<"-->motion model for state x ! \n" << std::endl;

	//get current observations and control information:
	MeasurementPackage::control_s     controls = measurements.control_s_;
	MeasurementPackage::observation_s observations = measurements.observation_s_;

	//run over all bel_x values (index represents the pose in x!):
	//for (int i=0; i< ...){


		/**************************************************************************
		 *  posterior for motion model
		**************************************************************************/

        // motion posterior:
        // used to set up the convlution
		//float posterior_motion = 0.0f;

		//loop over state space x_t-1 * same size as bel_x (Perform Convolution):
		//for (int j=0; j< ...){
			
			//TODO: Calculate transition probabilites using helpers.normpdf()
			// x: difference between bel_x index and state space index
			// mu: the movement from controls defined above
			// std: defined eariler
			
			//TODO: Calculate motion model
			// ADD the transition prob multiplied by the intial believe 
			// at state space index
			//posterior_motion +=
		//}

		//TODO: update = motion_model 
		// set as the posterior_motion
		//bel_x[i] =


	//};
		//TODO: normalize bel_x:
		

		//TODO: set initial believe to bel_x for next time
		//bel_x_init = 
	 
	 	//run over the whole state (index represents the pose in x!):
	
	for (int i=0; i< bel_x.size(); i++){

		float pose_i = float(i) ;
		/**************************************************************************
		 *  posterior for motion model
		**************************************************************************/

		// motion posterior:
		float posterior_motion = 0.0f;

		//loop over state space x_t-1 (convolution):
		for (int j=0; j< bel_x.size(); j++){
			float pose_j = float(j) ;
			
			
			float distance_ij = pose_i-pose_j;

			//transition probabilities:
			float transition_prob = helpers.normpdf(distance_ij,
                    								controls.delta_x_f,
                    								control_std) ;
			//motion model:
			posterior_motion += transition_prob*bel_x_init[j];
		}

		//update = motion_model
		bel_x[i] = posterior_motion ;


	};
		//normalize:
		bel_x = helpers.normalize_vector(bel_x);

		///set bel_x to bel_init:
		bel_x_init = bel_x;
	 

};
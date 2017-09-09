#include "PID.h"

#include <math.h>
#include <iostream>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;

	PID::p_error = 0.0;
	PID::d_error= 0.0;
	PID::i_error = 0.0;





	iter = 1;
	dp = {0.1*Kp, 0.1*Ki, 0.1*Kd};
	dp_size = 3;
	error = 0;
	best_error = std::numeric_limits<double>::max();
	cur_param_index = 0;

	added_dp, subtracted_dp = false;


	n_settle_steps = 100;
	n_eval_steps = 2000;





}

void PID::UpdateError(double cte) {


	if(iter == 1){
		p_error = cte; // so d_error is correct initially
	}


	d_error = cte - p_error; // error at time t - error at time (t-1)
	p_error = cte; // update to error at time t
	i_error += cte; // keep track of total error for the entire history


	if (iter % (n_settle_steps + n_eval_steps) > n_settle_steps ){
		error += pow(cte,2);
	}



	if ((iter % (n_settle_steps + n_eval_steps)) == 0 ) {
		//error = i_error/iter; //avg cte error is what twiddle will optimize on
		
	
		if (error < best_error){
			best_error = error;
			dp[cur_param_index] *= 1.1;

			cur_param_index = (cur_param_index +1) % dp_size ;
			added_dp, subtracted_dp = false;
		}


		if( !added_dp && !subtracted_dp ){
			AddChangesToParams(cur_param_index, dp[cur_param_index]);
			added_dp = true;
		}
		else if(added_dp && !subtracted_dp){
			AddChangesToParams(cur_param_index, -2*dp[cur_param_index]);
			subtracted_dp = true;
		}
		else{
			// when you have tried adding and subtracting dp and neither result in a lower error
			AddChangesToParams(cur_param_index, dp[cur_param_index]);
			dp[cur_param_index] *=  0.9;
			cur_param_index = (cur_param_index +1) % dp_size;
			added_dp, subtracted_dp = false;
		}
		 
		cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;        

	}
	iter++;

	cout<< iter << endl;

}

double PID::TotalError() {

	return (p_error + d_error + i_error);
}


double PID::CalcVal(int type){

	double value;

	if (type == 1) {
		value = -Kp * p_error - Ki * i_error - Kd * d_error;
	}
	else if (type ==2){
		value = 0.75 -Kp * p_error - Ki * i_error - Kd * d_error;
	}

	return	value;

}


void PID::AddChangesToParams(int index, double change ){

	if(index == 0){
		Kp += change;
	}
	else if(index == 1){
		Kd += change;
	}
	else if(index == 2){
		Ki += change;
	}
	else{
		cout << "AddChangesToParams: Out of Bounds";
	}

}

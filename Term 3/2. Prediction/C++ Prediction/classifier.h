#ifndef CLASSIFIER_H
#define CLASSIFIER_H
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <Eigen/Dense>

using namespace std;
using Eigen::ArrayXd;

class GNB {
public:

	vector<string> possible_labels = {"left","keep","right"};

	// vector<double> means;
	// vector<double> stds;


	ArrayXd left_means;
	ArrayXd left_sds;
	
	ArrayXd keep_means;
	ArrayXd keep_sds;
	
	ArrayXd right_means;
	ArrayXd right_sds;


	/**
  	* Constructor
  	*/
 	GNB();

	/**
 	* Destructor
 	*/
 	virtual ~GNB();

 	void train(vector<vector<double> > data, vector<string>  labels);

  	string predict(vector<double>);


private:


	double StandardDeviation(vector<double> samples);

	double Variance(vector<double> samples);

	double gaussian_prob(double obs, double mu, double sig);

};

#endif




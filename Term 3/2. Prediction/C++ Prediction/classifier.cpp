#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include "classifier.h"

/**
 * Initializes GNB
 */
GNB::GNB() {

}

GNB::~GNB() {}

void GNB::train(vector<vector<double>> data, vector<string> labels)
{

	/*
		Trains the classifier with N data points and labels.

		INPUTS
		data - array of N observations
		  - Each observation is a tuple with 4 values: s, d, 
		    s_dot and d_dot.
		  - Example : [
			  	[3.5, 0.1, 5.9, -0.02],
			  	[8.0, -0.3, 3.0, 2.2],
			  	...
		  	]

		labels - array of N labels
		  - Each label is one of "left", "keep", or "right".
	*/

	// each observation is s, d, s_dot, d_dot

	int num_vars = 4;
	vector<vector<double>> structured_data;

	for (int i = 0; i < labels.size(); i++) {
		structured_data.push_back(vector<double>());
	}

	for (int i = 0; i < labels.size(); i++) {
		for (int j = 0; j < possible_labels.size()) {

			if (labels[i] == possible_labels[j] ) {
				structured_data[j].push_back(data[i]);
			}
		}
	}

	for (int i = 0; i < possible_labels.size(), i++) {
		means[i].push_back(1.0 * accumulate(structured_data[i].begin(), structured_data[i].end(), 0LL) / structured_data[i].size());

		stds[i].push_back(StandardDeviation(structured_data[i]));
	}

}

string GNB::predict(vector<double> sample)
{
	/*
		Once trained, this method is called and expected to return 
		a predicted behavior for the given observation.

		INPUTS

		observation - a 4 tuple with s, d, s_dot, d_dot.
		  - Example: [3.5, 0.1, 8.5, -0.2]

		OUTPUT

		A label representing the best guess of the classifier. Can
		be one of "left", "keep" or "right".
		"""
		# TODO - complete this
	*/


	





	return this->possible_labels[1];

}







double StandardDeviation(vector<double> samples) 
{
	return sqrt(Variance(samples));
}

double Variance(vector<double> samples)
{
     int size = samples.size();

     double variance = 0;
     double t = samples[0];
     for (int i = 1; i < size; i++)
     {
          t += samples[i];
          double diff = ((i + 1) * samples[i]) - t;
          variance += (diff * diff) / ((i + 1.0) *i);
     }

     return variance / (size - 1);
}


double gaussian_prob(double obs, double mu, double sig)
{
	double num = (obs - mu)*(obs - mu);
	double denum = 2*sig*sig;
	double norm = 1 / sqrt(2*pi*sig**sig);

	return norm*exp(-num/denum);
}
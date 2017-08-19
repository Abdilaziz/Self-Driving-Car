/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "particle_filter.h"

using namespace std;

// useing the same random engine for the whole filter.
static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	num_particles = 200;

	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for (int i = 0; i < num_particles; i++) {
		Particle particle;

		particle.id = i;

		particle.x = dist_x(gen);

		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		
		particle.weight = 1.0;

		particles.push_back(particle);


	}


	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/


	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);



		for (int i = 0; i < num_particles; i++) {
			Particle tempParticle = particles[i];
				if (fabs(yaw_rate) < small_numb) {
					
					tempParticle.x += velocity*delta_t*cos(tempParticle.theta);
					tempParticle.y += velocity*delta_t*sin(tempParticle.theta);

				}
				else {
					double theta = tempParticle.theta;
					tempParticle.x += (velocity / yaw_rate)*(sin(theta + yaw_rate*delta_t) - sin(theta));

					tempParticle.y += (velocity / yaw_rate)*(cos(theta) - cos(theta + yaw_rate*delta_t));

					tempParticle.theta += yaw_rate*delta_t;
				}

				tempParticle.x += dist_x(gen);
				tempParticle.y += dist_y(gen);
				tempParticle.theta += dist_theta(gen);



				particles[i] = tempParticle;



	}
	


}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.


	// predicted is the map landmarks within the sensor range of a particle
	// observations is the measured landmark locations
	// Nearest Neighbour association (Landmark is associated with the clossest observation)

	
	for (int i = 0; i < observations.size(); i++) {

		LandmarkObs obs = observations[i];

		double min_distance = numeric_limits<double>::max();

		int map_id = -1;

		for (int j = 0; j < predicted.size(); j++) {

			LandmarkObs pred = predicted[j];

			double distance = dist(obs.x, obs.y, pred.x, pred.y );


			if (distance < min_distance) {
				min_distance = distance;
				map_id = pred.id;
			}

		}

		observations[i].id = map_id;
	}



}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html




	// change observations to map co-ordinates

	for (int i = 0; i < num_particles; i++) {

		// particles values

		double part_x = particles[i].x;
		double part_y = particles[i].y;
		double part_theta = particles[i].theta;

		
		//find the associated landmarks for each particle by checking if each landmark is within the sensors range

		vector<LandmarkObs> predictedLandmarks;

		for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {

			float landmark_x = map_landmarks.landmark_list[j].x_f;
			float landmark_y = map_landmarks.landmark_list[j].y_f;
			int landmark_id = map_landmarks.landmark_list[j].id_i;

			if (fabs(landmark_x - part_x) <= sensor_range  && fabs(landmark_y - part_y) <= sensor_range) {
				

				predictedLandmarks.push_back(LandmarkObs{ landmark_id, landmark_x, landmark_y });

			}

		}

		//transform observations from vehicle co-ordinates to map co-ordinates
		vector<LandmarkObs> transformed_observations;
		for (int j = 0; j < observations.size(); j++) {
			int id = observations[j].id;
			double x = cos(part_theta)*observations[j].x - sin(part_theta)*observations[j].y + part_x; // rotation and translation to map co-ordinates
			double y = sin(part_theta)*observations[j].x + cos(part_theta)*observations[j].y + part_y;
			transformed_observations.push_back(LandmarkObs{ id, x, y });
		}

		// associate each observation with a landmark
		dataAssociation(predictedLandmarks, transformed_observations);

		// re initialize the weight of each particle
		particles[i].weight = 1.0;

		// the weight of a particle is proportional with the observations that are associated
		for (int j = 0; j < transformed_observations.size(); j++) {

			int associated_measurement = transformed_observations[j].id;
			double obs_x = transformed_observations[j].x;
			double obs_y = transformed_observations[j].y;

			double pred_x;
			double pred_y;


			for (int k = 0; k < predictedLandmarks.size(); k++) {

				if (predictedLandmarks[k].id == associated_measurement) {
					pred_x = predictedLandmarks[k].x;
					pred_y = predictedLandmarks[k].y;
				}

			}

			// calculate weight with Multi-Variate Gaussian Distribution

			double std_landmark_x = std_landmark[0];
			double std_landmark_y = std_landmark[1];

			double gauss_norm = (2.0 * M_PI * std_landmark_x * std_landmark_y);

			double exponent = ((obs_x - pred_x)*(obs_x - pred_x)) / (2 * std_landmark_x*std_landmark_x) + ((obs_y - pred_y)*(obs_y - pred_y)) / (2 * std_landmark_y*std_landmark_y);

			particles[i].weight *= exp(-1.0*exponent)/ gauss_norm;


		}

	}
	
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution


	// fill the vector with all the particle weights

	for (int i = 0; i < num_particles; i++) {
		weights.push_back(particles[i].weight);
	}

	//double sum = accumulate(weights.begin(), weights.end(), 0);

	//int i = 0;
	//for (double a : weights) {
	//	weights[i] = a / sum;
	//	i++;
	//}	


	discrete_distribution<> random_index_distribution(0,num_particles);

	int index = random_index_distribution(gen);
	double beta = 0.0;

	double max_weight = *max_element( begin(weights), end(weights));

	vector<Particle> resampled_particles;

	discrete_distribution<> random_beta_increment(0.0, max_weight);

	for (int i = 0; i < num_particles; i++) {
		beta += random_beta_increment(gen)*2.0;

		while (beta > weights[index]) {
			beta -= weights[index];
			index = (index + 1) % num_particles;
		}
		resampled_particles.push_back(particles[index]);
	}

	particles = resampled_particles;


}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

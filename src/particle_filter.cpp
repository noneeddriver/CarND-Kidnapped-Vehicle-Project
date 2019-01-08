/*
 * particle_filter.cpp
 *
 *   Created on: Jan 02, 2019
 *      Author: Pengmian Yan
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
#include <chrono>
#include <limits>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
	num_particles = 100; //set the number of particles

	// construct a trivial random generator engine from a time-based seed:
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	default_random_engine gen (seed);

	for (unsigned int i = 0; i < num_particles; i++) {
		Particle particle;
		normal_distribution<double> dist_x(x,std[0]), dist_y(y,std[1]), dist_theta(theta,std[2]); //build distributions for positions and yaw
		particle.id = i;
		particle.x =  dist_x(gen);
		particle.y =  dist_y(gen);
		particle.theta =  dist_theta(gen);
		particle.weight =  1.0;
		particles.push_back(particle);	
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	
	// construct a trivial random generator engine from a time-based seed:
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	default_random_engine gen (seed);
	
	for (unsigned int i = 0; i < particles.size(); i++) {
		double x, y, theta;
		theta = particles[i].theta + yaw_rate * delta_t;
		if (yaw_rate > 0.001 || yaw_rate < -0.001) {
			x = particles[i].x + velocity * (sin(theta) - sin(particles[i].theta)) / yaw_rate;
			y = particles[i].y + velocity * (cos(particles[i].theta) - cos(theta)) / yaw_rate;
		}
		else {
			x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
			y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
		}
		
		//add random Gaussian noise
		normal_distribution<double> dist_x(x,std_pos[0]),dist_y(y,std_pos[1]), dist_theta(theta,std_pos[2]);
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(Particle &particle, std::vector<LandmarkObs> predicted) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	
	for (unsigned int i = 0; i < particle.sense_x.size(); i++) {
		double distance_min = numeric_limits<double>::infinity();
		double obs_x = particle.sense_x[i]; //observation in map coordinates
		double obs_y = particle.sense_y[i];
		unsigned int id = 1000; // id of predicted landmark, which has the smallest distence to observation
		for (unsigned int j = 0; j < predicted.size(); j++) {
			double distance_to_obs = dist(obs_x, obs_y, predicted[j].x, predicted[j].y);
			if (distance_to_obs < distance_min) {
				distance_min = distance_to_obs;
				id = predicted[j].id;
			}
		}
		particle.associations.push_back(id); 
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
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
	
	double weight_sum = 0.0;   //summe of particles' weights
	
	for (unsigned int i = 0; i < particles.size(); i++) {
		particles[i].associations.clear();
		particles[i].sense_x.clear();
		particles[i].sense_y.clear();
		//Homogenous transformation: transfer the observations from VEHICLE'S coordinate system to MAP'S coordinate system
		for (unsigned int j = 0; j < observations.size(); j++) {
			double xm = 0.0, ym = 0.0; //map coordinates
			xm =  particles[i].x + cos(particles[i].theta) * observations[j].x - sin(particles[i].theta) * observations[j].y;
			ym =  particles[i].y + sin(particles[i].theta) * observations[j].x + cos(particles[i].theta) * observations[j].y;
		    particles[i].sense_x.push_back(xm);
			particles[i].sense_y.push_back(ym);
		}

		//get the landmarks within the sensor range to the particle
		vector<LandmarkObs> predicted;// Landmark list in sensor range
		for (unsigned int k = 0; k < map_landmarks.landmark_list.size(); k++) {
			LandmarkObs predict;  //landmark in sensor range
			double distance_to_pf;  //distance from Landmark to particle
			distance_to_pf = dist(map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f, particles[i].x, particles[i].y);
			if (distance_to_pf <= sensor_range) {
				predict.id =  map_landmarks.landmark_list[k].id_i;
				predict.x =  map_landmarks.landmark_list[k].x_f;
				predict.y =  map_landmarks.landmark_list[k].y_f;
				predicted.push_back(predict);
			}
		}

		//associate the observations with landmarks
		dataAssociation(particles[i], predicted);
		predicted.clear();
		
		//compute the weights of particles
		double weight = 1.0;
		for (unsigned int p = 0; p < observations.size(); p++) {
			double mu_x = map_landmarks.landmark_list[particles[i].associations[p]-1].x_f; //x_position of landmark
			double mu_y = map_landmarks.landmark_list[particles[i].associations[p]-1].y_f; //y_position of landmark
			double gauss_norm= (1/(2 * M_PI * std_landmark[0] * std_landmark[1]));  //calculate normalization term
			double exponent= ((particles[i].sense_x[p] - mu_x) * (particles[i].sense_x[p] - mu_x)) / (2 * std_landmark[0] * std_landmark[0]) + ((particles[i].sense_y[p] - mu_y) * (particles[i].sense_y[p] - mu_y)) / (2 * std_landmark[1] * std_landmark[1]);   //calculate exponent
			weight = weight * gauss_norm * exp(-exponent); //calculate weight using normalization terms and exponent
		}
		particles[i].weight = weight; //update the weight of particles
		weight_sum += particles[i].weight;
	}

	//normalize the particle weights
	if (weight_sum != 0) {  
		for (unsigned int i = 0; i < particles.size(); i++) {
			particles[i].weight = particles[i].weight / weight_sum;
		}
	}
	else
		cout<<"weight_sum = 0"<<endl; // in case particles are all very bad, so all weights are zero
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    
	//use the discrete_distribution to generate random index accor to weights
	vector<double> weights; //put the particle weights into a vector
	vector<Particle> particles_new; // declare the new particle list
	
	//construct a high quality random generator engine
	std::random_device rd;
    std::mt19937 gen(rd());
	
	//store the weights of particles in a list "weights", which will be used for discrete_distribution as a base of frequence
	for (unsigned int i = 0; i < particles.size(); i++) {
		weights.push_back(particles[i].weight);
	}
	
	//build the distribution for random index; the frequences are proportional to the weights
	discrete_distribution<int> dist_index(weights.begin(), weights.end());

	//select  new partilces
	for (unsigned int i = 0; i < particles.size(); i++) {
		particles_new.push_back(particles[dist_index(gen)]);
	}
	particles = particles_new; 
	particles_new.clear();
	weights.clear();
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

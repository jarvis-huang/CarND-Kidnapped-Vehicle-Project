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
#include <limits>
#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    particles.resize(num_particles);
    weights.resize(num_particles, 1.0);
    double std_x = std[0];
    double std_y = std[1];
    double std_theta = std[2];
    std::normal_distribution<double> dist_x(0, std_x);
    std::normal_distribution<double> dist_y(0, std_y);
    std::normal_distribution<double> dist_theta(0, std_theta);
    std::default_random_engine gen;
    int id=0;
    for (auto& particle: particles) {
        particle.x = x + dist_x(gen);
        particle.y = y + dist_y(gen);
        particle.theta = theta + dist_theta(gen);
        particle.weight = 1.0;
        particle.id = id++;
    }
    
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    
    // I will use the Bicycle Motion Model presented in class
    double std_x = std_pos[0];
    double std_y = std_pos[1];
    double std_theta = std_pos[2];
    std::normal_distribution<double> dist_x(0, std_x);
    std::normal_distribution<double> dist_y(0, std_y);
    std::normal_distribution<double> dist_theta(0, std_theta);
    std::default_random_engine gen;
    
    for (auto& particle: particles) {
        double x, y, theta;
        x = particle.x;
        y = particle.y;
        theta = particle.theta;
        particle.x += velocity/yaw_rate*(sin(theta+yaw_rate*delta_t)-sin(theta)) + dist_x(gen);
        particle.y += velocity/yaw_rate*(cos(theta)-cos(theta+yaw_rate*delta_t)) + dist_y(gen);
        particle.theta += yaw_rate*delta_t + dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
    
    // Nearest neighbor algorithm => associations
    associations.resize(predicted.size());
    for (unsigned int i=0; i<predicted.size(); i++)
    {
        auto& pred = predicted[i];
        double min_dist = std::numeric_limits<double>::max();
        int matched_id = -1;
        for (unsigned int j=0; j<observations.size(); j++)
        {
            auto& obs = observations[i];
            double this_dist = dist(obs.x, obs.y, pred.x, pred.y);
            if (this_dist<min_dist) {
                matched_id = j;
                min_dist = this_dist;
            }
        }
        if (matched_id==-1) {
            std::cerr << "dataAssociation failed!" << std::endl;
            exit(1);
        } else {
            associations[i] = matched_id;
        }
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
    weights.clear();
    for (auto& particle: particles)
    {
        double x, y, theta, weight;
        x = particle.x;
        y = particle.y;
        theta = particle.theta;
        weight = particle.weight;
        
        // Transform observations => MAP coordinate
        // P_w = [cos -sin  * P_v + [v.x
        //        sin  cos]          v.y]
        std::vector<LandmarkObs> trans_observations(observations.size());
        for (unsigned int i=0; i<observations.size(); i++)
        {
            double obs_x = observations[i].x; 
            double obs_y = observations[i].y;
            theta = particle.theta;
            trans_observations[i].x = cos(theta)*obs_x - sin(theta)*obs_y + x;
            trans_observations[i].y = sin(theta)*obs_x + cos(theta)*obs_y + y;
        }
        
        // Compute predicted measurements
        std::vector<LandmarkObs> pred_observations;
        for (auto& lm: map_landmarks.landmark_list) {
            if (dist(x, y, lm.x_f, lm.y_f)<sensor_range) {
                LandmarkObs lmo;
                lmo.x = lm.x_f;
                lmo.y = lm.y_f;
                pred_observations.push_back(lmo);
            }
        }
        
        // call dataAssociation
        dataAssociation(pred_observations, trans_observations)；
        
        // compute new weight using mult-variate Gaussian distribution
        double std_x = std_landmark[0];
        double std_y = std_landmark[1];
        double var_x = std_x*std_x;
        double var_y = std_y*std_y;
        double w = 1;
        for (unsigned int i=0; i<pred_observations.size(); i++) {
            LandmarkObs& pred = pred_observations[i];
            int match_id = associations[i];
            LandmarkObs& obs = trans_observations[match_id];
            double error_x = fabs(pred.x - obs.x);
            double error_y = fabs(pred.y - obs.y);
            double num = std::exp(-0.5*(error_x*error_x/var_x + error_y*error_y/var_y));
            double denom = 2*M_PI*std_x*std_y;
            w *= num/denom;
        }
        
        // update particle and weights
        particle.weight = w;
        weights.push_back(w);
    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    
    std::discrete_distribution<double> dist(weights);
    std::default_random_engine gen;
    std::vector<Particle> new_particles(num_particles);
    for (unsigned int i=0; i<num_particles; i++)
    {
        int pick = dist(gen);
        new_particles[i] = particles[pick];
        new_particles[i].weight = 1;
    }
    particles.swap(new_particles);
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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

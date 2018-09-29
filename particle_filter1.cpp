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

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std_init[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    num_particles = 1000;

    //grab standard deviations for readability
    double std_x = std_init[0];
    double std_y = std_init[1];
    double std_theta = std_init[2];

    //make generator
    default_random_engine gen;

    //create normal distributions to draw from for x, y, and theta
    normal_distribution<double> dist_x(x, std_x);
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);

    //go through each particle and get initial position and theta
    for(int i = 0; i < num_particles; i++){
        double sample_x, sample_y, sample_theta;

        //get sample x, y, and theta values for each particle
        sample_x = dist_x(gen);
        sample_y = dist_y(gen);
        sample_theta = dist_theta(gen);

        //create particle and assign values
        Particle new_particle;

        new_particle.id = i;
        new_particle.x = sample_x;
        new_particle.y = sample_y;
        new_particle.theta = sample_theta;
        new_particle.weight = 1.0;

        // push new particle to vector
        particles.push_back(new_particle);

        //update weights variable
        weights.push_back(1.0);

    }
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    //grab standard deviations for readability

    double std_x = std_pos[0];
    double std_y = std_pos[1];
    double std_yaw = std_pos[2];

    default_random_engine gen;

    normal_distribution<double> dist_x(0, std_x);
    normal_distribution<double> dist_y(0, std_y);
    normal_distribution<double> dist_yaw(0, std_yaw);

    // cycle through particles
    for(int part_i = 0; part_i < num_particles; part_i++){

        //predict particle states
        if(fabs(yaw_rate) < 0.001){
            //predict if yaw_rate is 0
            particles[part_i].x = particles[part_i].x + velocity * delta_t * cos(particles[part_i].theta);
            particles[part_i].y = particles[part_i].y + velocity * delta_t * sin(particles[part_i].theta);
            particles[part_i].theta = particles[part_i].theta;
        }
        else {
            //predict normally if yaw rate is not zero
            particles[part_i].x = particles[part_i].x + velocity * (sin(particles[part_i].theta + yaw_rate*delta_t) - sin(particles[part_i].theta))/yaw_rate;
            particles[part_i].y = particles[part_i].y + velocity * (cos(particles[part_i].theta) - cos(particles[part_i].theta + yaw_rate*delta_t))/yaw_rate;
            particles[part_i].theta = particles[part_i].theta + yaw_rate*delta_t;
        }
        //add uncertainty
        particles[part_i].x += dist_x(gen);
        particles[part_i].y += dist_y(gen);
        particles[part_i].theta += dist_yaw(gen);

    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

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

	//convert vehicles observations to map coordination by rotation and translation
	//cycle through each particle and update weight
	for(int i = 0; i < num_particles; i++){


        // get particles position in map coordinates
        double part_x = particles[i].x;
        double part_y = particles[i].y;
        double part_theta = particles[i].theta;


        //TRANSFORM STEP
        //define vector to hold the coordinates in map space of all of the observations in car space
        std::vector<LandmarkObs> trans_obs;
        //cycle through observations and transform them to map space
        //transform all of the observations into map space
        //with respect to particle
        for(unsigned int tran = 0; tran < observations.size(); tran++){
                //get x and y components for each observation
                double obs_x = observations[tran].x;
                double obs_y = observations[tran].y;

                double x_map, y_map;
                //calculate tranformed x and y observation values
                x_map = part_x + (cos(part_theta) * obs_x) - (sin(part_theta) * obs_y);
                y_map = part_y + (sin(part_theta) * obs_x) + (cos(part_theta) * obs_y);

                // append x_map and y_map to trans_obs vector
                LandmarkObs temp_obs;
                temp_obs.x = x_map;
                temp_obs.y = y_map;
                trans_obs.push_back(temp_obs);
        }



        //ASSOCIATION STEP
        //get association for each transformed observation
        //take all of the transformed observations
        //find closest map landmark to observation and store in obs_associate
        std::vector<LandmarkObs> obs_associate;

        for(unsigned int obsa = 0; obsa < trans_obs.size(); obsa++){


                //get values for readability
                double x_trans = trans_obs[obsa].x;
                double y_trans = trans_obs[obsa].y;
                double x_map = map_landmarks.landmark_list[0].x_f;
                double y_map = map_landmarks.landmark_list[0].y_f;

                LandmarkObs closest;
                double temp_value;
                //calculate initial distance to start off
                temp_value = dist(x_trans, y_trans, x_map, y_map);
                closest.x = x_map;
                closest.y = y_map;

                //take the transformed map location from above and find the closest map landmark
                for(unsigned int j = 1; j < map_landmarks.landmark_list.size(); j++){
                    //get values for readability
                    double x_map = map_landmarks.landmark_list[j].x_f;
                    double y_map = map_landmarks.landmark_list[j].y_f;

                    //calculate distance and check to see if its smaller than the last
                    double next_value = dist(x_trans, y_trans, x_map, y_map);
                    if(next_value < temp_value){
                        temp_value = next_value;
                        closest.x = x_map;
                        closest.y = y_map;
                }
            }

            // append smallest map landmark to obs_associate
            obs_associate.push_back(closest);
        }




        //UPDATE STEP
        //update both particle weights and weight vector
        //go though each transformed observation and its associated landmark
        //use these two to update particles weight
        //get std_x and std_y
        double std_x = std_landmark[0];
        double std_y = std_landmark[1];
        double new_weight = 1.0;

        //go through each observation and and update weight according to closest landmark measurement
        for(unsigned int update_i = 0; update_i < trans_obs.size(); update_i++){


            //define variables
            double tran_x = trans_obs[update_i].x;
            double tran_y = trans_obs[update_i].y;
            double mu_x = obs_associate[update_i].x;
            double mu_y = obs_associate[update_i].y;

            //calculate normalization term
            double gauss_norm = (1 / (2.0 * M_PI * std_x * std_y));

            //define terms for readability
            double std_x2 = std_x * std_x;
            double std_y2 = std_y * std_y;
            double term1 = tran_x - mu_x;
            term1 = term1 * term1;
            double term2 = tran_y - mu_y;
            term2 = term2 * term2;
            //calculate exponent term
            double exponent = (term1 / (2 * std_x2)) + (term2 / (2 * std_y2));

            double temp = gauss_norm * exp(-exponent);
            //update new weight

            new_weight = new_weight * temp;
            }

        //update particles weight with new weight
        particles[i].weight = new_weight;
        weights[i] = new_weight;

	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution


    //new list of particles
    std::vector<Particle> new_particles;
    //create random generator uniform distribution
    //get other values
    default_random_engine generator;
    uniform_real_distribution<double> distribution(0.0, 1.0);
    int index = distribution(generator) * num_particles;

    double beta = 0.0;
    //get max weight value
    double mw = *max_element(weights.begin(), weights.end());


    //Go through each particle
    //use the weights to resample
    //for each particle that is samples append to new_particles vector
    //replace old particle vector with new_particles vector
    for(int samp_i = 0; samp_i < num_particles; samp_i++){


        //update beta
        beta = beta + (distribution(generator) * 2.0 * mw);
        //sample
        while(beta > weights[index]){

                beta = beta - weights[index];
                index = (index + 1) % num_particles;
        }

        //define new particle
        Particle pick_particle;
        //get values from old particle
        pick_particle.id = particles[index].id;
        pick_particle.x = particles[index].x;
        pick_particle.y = particles[index].y;
        pick_particle.theta = particles[index].theta;
        pick_particle.weight = particles[index].weight;

        //push new particle to new list
        new_particles.push_back(pick_particle);
    }

    // make old particle list the new resampled particle list
    particles = new_particles;

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

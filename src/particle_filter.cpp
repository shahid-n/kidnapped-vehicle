/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
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
static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */

  num_particles = 100;

  normal_distribution<double> N_x(x, std[0]);
  normal_distribution<double> N_y(y, std[1]);
  normal_distribution<double> N_theta(theta, std[2]);

  for (int i = 0; i < num_particles; ++i) {
    Particle p;

    p.id = i;
    p.x = N_x(gen);
    p.y = N_y(gen);
    p.theta = N_theta(gen);
    p.weight = 1.0;

    particles.push_back(p);
    weights.push_back(p.weight);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  normal_distribution<double> N_x(0, std_pos[0]);
  normal_distribution<double> N_y(0, std_pos[1]);
  normal_distribution<double> N_theta(0, std_pos[2]);

  for (int i = 0; i < num_particles; ++i) {
//  Division by zero check; execute the else branch if yaw_rate == 0.0
    if (fabs(yaw_rate) > 0.0) {
      particles[i].x += velocity/yaw_rate*(sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
      particles[i].y += velocity/yaw_rate*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
      particles[i].theta += yaw_rate*delta_t;
    }
    else {
      particles[i].x += velocity*cos(particles[i].theta)*delta_t;
      particles[i].y += velocity*sin(particles[i].theta)*delta_t;
    }

//  Add zero-mean Gaussian noise
    particles[i].x += N_x(gen);
    particles[i].y += N_y(gen);
    particles[i].theta += N_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will
   *   probably find it useful to implement this method and use it as a helper
   *   during the updateWeights phase.
   */

//Initialise min_dist to the max possible value for its data type
  double min_dist = numeric_limits<double>::max();

  for (unsigned int i = 0; i < observations.size(); ++i) {
    int nearest_LM_id = -1;

  for (unsigned int j = 0; j < predicted.size(); ++j) {
    double curr_dist = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

    if (curr_dist < min_dist) {
      min_dist = curr_dist;
      nearest_LM_id = predicted[j].id;
    }
  }

  observations[i].id = nearest_LM_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian
   *   distribution. You can read more about this distribution here:
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system.
   *   Your particles are located according to the MAP'S coordinate system.
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  double wt_sum = 0.0;
  vector<LandmarkObs> T_obsv(observations.size());

  for (int i = 0; i < num_particles; ++i) {

//  Transform observations from vehicle coordinates to map coordinates
    for (unsigned int j = 0; j < observations.size(); ++j) {
      T_obsv[j].id = j;
      T_obsv[j].x = particles[i].x + cos(particles[i].theta)*observations[j].x\
                    - sin(particles[i].theta)*observations[j].y;
      T_obsv[j].y = particles[i].y + sin(particles[i].theta)*observations[j].x\
                    + cos(particles[i].theta)*observations[j].y;
    }

//  Filter the landmarks and keep only those within sensor range of each particle; collect them in the prediction vector
    vector<LandmarkObs> pred_LM;
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); ++j) {
      Map::single_landmark_s curr_LM = map_landmarks.landmark_list[j];
      if (dist(particles[i].x, particles[i].y, curr_LM.x_f, curr_LM.y_f) <= sensor_range) {
        pred_LM.push_back(LandmarkObs {curr_LM.id_i, curr_LM.x_f, curr_LM.y_f});
      }
    }

//  Associate observations with predicted landmarks (nearest neighbour approach)
    dataAssociation(pred_LM, T_obsv);

//  Initialise weights to 1.0 before applying iterative multiplication
    particles[i].weight = 1.0;

    double s_x = std_landmark[0];
    double s_y = std_landmark[1];

    for (unsigned int j = 0; j < T_obsv.size(); ++j) {
      double obsv_x = T_obsv[j].x;
      double obsv_y = T_obsv[j].y;
      double prob = 1.0;

      for (unsigned int k = 0; k < pred_LM.size(); ++k) {
        double pr_x = pred_LM[k].x;
        double pr_y = pred_LM[k].y;

        if (T_obsv[j].id == pred_LM[k].id) {
//        Multivariate Gaussian distribution
          prob = 1/(2*M_PI*s_x*s_y)*exp(-pow((obsv_x - pr_x), 2)/(2*s_x*s_x) - pow((obsv_y - pr_y), 2)/(2*s_y*s_y));
          particles[i].weight *= prob;
        }
      }
    }

    wt_sum += particles[i].weight;
  }

//Normalise the weights
  for (unsigned int i = 0; i < particles.size(); ++i) {
    particles[i].weight /= wt_sum;
    weights[i] = particles[i].weight;
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

vector<Particle> resampled_particles(particles.size());

//Generate random particle index
  uniform_int_distribution<int> rnd_particle_idx(0, num_particles - 1);

  int idx = rnd_particle_idx(gen);

  double beta = 0.0;

  double max_weight = *max_element(weights.begin(), weights.end());

//Resampling wheel
  for (unsigned int i = 0; i < particles.size(); ++i) {
    uniform_real_distribution<double> rnd_weight(0.0, max_weight);
    beta += 2.0*rnd_weight(gen);

    while (beta > weights[idx]) {
      beta -= weights[idx];
      idx = (idx + 1) % num_particles;
    }

    resampled_particles[i] = particles[idx];
  }

  particles = resampled_particles;
}

void ParticleFilter::SetAssociations(Particle& particle,
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

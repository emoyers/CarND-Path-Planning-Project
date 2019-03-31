
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include "Eigen-3.3/Eigen/Dense"
#include "macros.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using std::map;
using std::cout;
using std::endl;


vector<double> JMT_get_alphas(vector<double> start, vector<double> end, double T);

double logistic(double x);
double nearest_approach(vector<double> trajectory_s, vector<double> trajectory_d, vector<vector<double>> other_vehicle_trajectory);
double nearest_approach_to_any_vehicle(vector<double> trajectory_s, vector<double> trajectory_d, map<double, vector<vector<double>> > other_cars_predictions);
double nearest_approach_to_same_lane_vehicle(vector<double> trajectory_s, vector<double> trajectory_d, map<double, vector<vector<double>> > other_cars_predictions);
double calculate_trajectory_cost(vector<vector<double>> trajectory, map<double, vector<vector<double>> > other_cars_predictions);

double collision_cost(double closest_vehicle_distance);
double fast_2_target_cost(vector<double> trajectory_s);
double closer_other_cars_any_lane_cost(double closest_vehicle_distance);
double closer_infront_car_same_lane_cost (double closest_vehicle_distance);
double go_2_middle_lane_cost(vector<double> trajectory_d);
double left_over_right_cost(vector<double> trajectory_d);
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iostream>
#include "macros.h"

using std::vector;
using std::set;
using std::map;
using std::string;

class Vehicle{

public:

	double car_x;
	double car_y;
	double car_s;
	double car_d;
	double s_dot;
	double d_dot;
	double s_dot_dot;
	double d_dot_dot;
	double car_vx;
	double car_vy;
	double car_yaw;
	double car_speed;
	double ref_velocity;
	int actual_lane;

	set<string> possible_next_states;

	map<double, vector<double>> previous_others_cars_d;

	Vehicle();

	Vehicle(double ref_vel, int lane);

	virtual ~Vehicle();


	void set_parameters(double x, double y, double s, double d, double yaw, double speed);

	void set_other_car_parameters(double x, double y, double s, double d, double speed);

	void set_parameters_S_D(double s, double d, double car_s_dot, double car_d_dot, double car_s_dot_dot, double car_d_dot_dot);

	vector<vector<double>> other_cars_predict(int num_predicted_points, int start);

	void set_possible_next_states(bool car_right, bool car_left);

	vector<vector<double>> calculate_target_s_and_d(string state, int start, map<double, vector<vector<double>> > other_cars_prediction);
};
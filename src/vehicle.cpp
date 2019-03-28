#include "vehicle.h"

#define SIZE_OF_VECTOR_OTHER_CARS_PREDICTION 20
#define delta_time 0.02

Vehicle::Vehicle(){}

Vehicle::Vehicle(double ref_vel, int lane){
	this->ref_velocity = ref_vel;
	this->actual_lane = lane;
}

Vehicle::~Vehicle() {}


void Vehicle::set_parameters(double x, double y, double s, double d, double yaw, double speed){
	this->car_x = x;
	this->car_y = y;
	this->car_s = s;
	this->car_d = d;
	this->car_yaw = yaw;
	this->car_speed =  speed;
}

void Vehicle::set_other_car_parameters(double x, double y, double s, double d, double speed){
	this->car_x = x;
	this->car_y = y;
	this->car_s = s;
	this->car_d = d;
	this->car_speed =  speed;
}

vector<vector<double>> Vehicle::other_cars_predict(int num_predicted_points){
	vector<double> s_predictions;
	vector<double> d_predictions;
	for (int i = 0; i < num_predicted_points; ++i)
	{
		double new_s = this->car_s + (i * delta_time) * this->car_speed;
		s_predictions.push_back(new_s);
		d_predictions.push_back(this->car_d);
	}

	vector<vector<double>> result = {s_predictions, d_predictions};

	return result;
}

void Vehicle::set_parameters_S_D(double s, double d, double car_s_dot, double car_d_dot, double car_s_dot_dot, double car_d_dot_dot){

	this->car_s = s;
	this->car_d = d;
	this->s_dot = car_s_dot;
	this->d_dot = car_d_dot;
	this->s_dot_dot = car_s_dot_dot;
	this->d_dot_dot = car_d_dot_dot;
}

#include "vehicle.h"

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

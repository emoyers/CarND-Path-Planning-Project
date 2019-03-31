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

void Vehicle::set_other_car_parameters(double x, double y, double s, double d, double speed){
	this->car_x = x;
	this->car_y = y;
	this->car_s = s;
	this->car_d = d;
	this->car_speed =  speed;
}

vector<vector<double>> Vehicle::other_cars_predict(int num_predicted_points, int start){
	vector<double> s_predictions;
	vector<double> d_predictions;
	for (int i = 0; i < num_predicted_points; ++i)
	{
		double new_s = this->car_s + ( (start + i) * delta_time) * this->car_speed;
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

void Vehicle::set_possible_next_states(bool car_right, bool car_left){
	this->possible_next_states.clear();

	this->possible_next_states.insert("Front");

	if( car_right==false && this->car_d<8){
	 	this->possible_next_states.insert("Right");
	 }

	if( car_left==false && this->car_d>4){
	 	this->possible_next_states.insert("Left");
	 }
}

vector<vector<double>> Vehicle::calculate_target_s_and_d(string state, int start, map<double, vector<vector<double>> > other_cars_prediction){
	int num_element2calculate = num_next_points_calulated - start;
	this->actual_lane = (int)(this->car_d / 4); 

	double target_s = 0.0;
	double target_s_dot =  (MAX_VELOCITY/2.24);/*2.24 division to convert mph to m/s*/
	double target_s_dot_dot = 0.0;

	double target_d = 0.0;
	double target_d_dot = 0.0;
	double target_d_dot_dot = 0.0;

	target_s = this->car_s + (this->s_dot + target_s_dot)/2 * num_element2calculate*0.02;
	
	/*get target d*/
	if(state == "Front"){
		target_d = this->actual_lane* 4 + 2;
	}
	else if(state == "Right"){
		target_d = (this->actual_lane + 1) * 4 + 2;
	}
	else if(state == "Left"){
		target_d = (this->actual_lane - 1) * 4 + 2;
	}

	/*calculate closer vehicle in target lane*/
	int target_lane = (int)(target_d / 4);
	
	double closer_vehicle_s = 100000.0;
	double closer_vehicle_speed = 0.0;
	for(auto car_pred : other_cars_prediction){
		int other_car_lane = (int)(car_pred.second[1][0]/4);


		if(other_car_lane == target_lane){
			double first_s = car_pred.second[0][0];
			double last_s = car_pred.second[0][car_pred.second[0].size()-1];
        	double other_car_speed = (last_s-first_s)/((car_pred.second[0].size())*0.02);

        	if((last_s < closer_vehicle_s) && (first_s > this->car_s)){
        		closer_vehicle_s = last_s;
        		closer_vehicle_speed = other_car_speed;
        	}
		}

	}

	double delta_s = (closer_vehicle_s - target_s);
	if( delta_s < FREE_SPACE) {
		target_s_dot = closer_vehicle_speed;

		if(delta_s < (0.5* FREE_SPACE)){
			target_s_dot -= 1;
		}

		target_s = closer_vehicle_s - FREE_SPACE;
	}

	
	return {{target_s, target_s_dot, target_s_dot_dot}, {target_d, target_d_dot, target_d_dot_dot},{closer_vehicle_speed}};
}

vector<vector<double>> Vehicle::get_trajectory(vector<vector<double>> s_d_final, int start){
  vector<double> s_trajectory;
  vector<double> d_trajectory;

  double final_time = (num_next_points_calulated - start) * 0.02;

  vector<double> alphas_s = JMT_get_alphas({this->car_s, this->s_dot, this->s_dot_dot},s_d_final[0], final_time);
  vector<double> alphas_d = JMT_get_alphas({this->car_d, this->d_dot, this->d_dot_dot},s_d_final[1], final_time);

  int number_of_samples = num_next_points_calulated - start;
  for( int i=0; i< number_of_samples;i++){

  	double time_i = (i+1) * (final_time/number_of_samples);
  	double temp_s = 0.0;
  	double temp_d = 0.0;

  	for(int j=0; j< alphas_s.size(); j++){
  		temp_s += alphas_s[j]*pow(time_i,j);
  	}

  	for(int j=0; j< alphas_d.size(); j++){
  		temp_d += alphas_d[j]*pow(time_i,j);
  	}

  	s_trajectory.push_back(temp_s);
  	d_trajectory.push_back(temp_d);
  }

  return {s_trajectory, d_trajectory};
}
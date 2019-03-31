
#include "costs_and_JMT.h"

#define COLLISION_WEIGHT 99999
#define FAST_2_TARGET_WEIGHT 10000
#define CLOSER_OTHER_CARS_ANY_LANE_WEIGHT 10
#define CLOSER_INFRONT_CAR_SAME_LANE_WEIGHT 1000
#define GO_2_MIDDLE_LANE_WEIGHT 100
#define LEFT_OVER_RIGHT_WEIGHT 10

vector<double> JMT_get_alphas(vector<double> start, vector<double> end, double T) {

   vector <double>result=start;
   result[2] = 0.5*result[2]; 
   
   MatrixXd temp_mat(3,3);
   temp_mat(0,0) = T*T*T;
   temp_mat(0,1) = temp_mat(0,0)*T;
   temp_mat(0,2) = temp_mat(0,1)*T;
   temp_mat(1,0) = 3*T*T;
   temp_mat(1,1) = 4*temp_mat(0,0);
   temp_mat(1,2) = 5*temp_mat(0,1);
   temp_mat(2,0) = 6*T;
   temp_mat(2,1) = 4*temp_mat(1,0);
   temp_mat(2,2) = 5*temp_mat(1,1);
   
   VectorXd temp_vect(3);
   temp_vect(0) = end[0]-(start[0]+start[1]*T+0.5*start[2]*T*T);
   temp_vect(1) = end[1]-(start[1]+start[2]*T);
   temp_vect(2) = end[2]-start[2];
   
   temp_vect = temp_mat.inverse()*temp_vect;

    for (int i=0; i<3; i++){
        result.push_back(temp_vect(i));
    }
    
  return result;
}

double logistic(double x){
  return 2.0 / (1 + exp(-x)) - 1.0;
}

double nearest_approach(vector<double> trajectory_s, vector<double> trajectory_d, vector<vector<double>> other_vehicle_trajectory){
  double closest = 999999;

  for(int i = 0; i < trajectory_s.size(); i++){

    double distance2closest = sqrt(pow(trajectory_s[i] - other_vehicle_trajectory[0][i], 2) + pow(trajectory_d[i] - other_vehicle_trajectory[1][i], 2));
    if(distance2closest<closest) closest = distance2closest;
  }

  return closest;
}

double nearest_approach_to_any_vehicle(vector<double> trajectory_s, vector<double> trajectory_d, map<double, vector<vector<double>> > other_cars_predictions){
  double closest = 999999;

  for (auto other_car : other_cars_predictions) {

    double distance2closest = nearest_approach(trajectory_s, trajectory_d, other_car.second);
    
    if (distance2closest < closest) {

      closest = distance2closest;

    }

  }
  return closest;
}

double nearest_approach_to_same_lane_vehicle(vector<double> trajectory_s, vector<double> trajectory_d, map<double, vector<vector<double>> > other_cars_predictions){
  double closest = 999999;

  for (auto other_car : other_cars_predictions) {

    double last_d_evo = trajectory_d[trajectory_d.size() - 1];
    int last_lane_evo = (int)(last_d_evo / 4);
    double last_d_other_car = other_car.second[1][other_car.second[1].size()-1];
    int last_lane_other_car = (int)(last_d_other_car / 4);

    if(last_lane_evo == last_lane_other_car){

      double distance2closest = nearest_approach(trajectory_s, trajectory_d, other_car.second);
      
      if (distance2closest < closest) {

        closest = distance2closest;

      }      

    }



  }
  return closest;
}

double calculate_trajectory_cost(vector<vector<double>> trajectory, map<double, vector<vector<double>> > other_cars_predictions){
  vector<double> trajectory_s = trajectory[0];
  vector<double> trajectory_d = trajectory[1];
  //cout<<"----------------LANE: "<<(int)(trajectory_d[trajectory_d.size()-1]/4)<<" ------------------------------"<<endl;
  double closets_car_any_lane = nearest_approach_to_any_vehicle(trajectory_s, trajectory_d, other_cars_predictions);
  //cout<<"closets_car_any_lane : "<< closets_car_any_lane<<endl;
  double closets_car_same_lane = nearest_approach_to_same_lane_vehicle(trajectory_s, trajectory_d, other_cars_predictions);
  //cout<<"closets_car_same_lane : "<< closets_car_same_lane<<endl;

  double cost = 0.0;

  double cost_collision = collision_cost(closets_car_any_lane) * COLLISION_WEIGHT;
  double cost_fast_2_target = fast_2_target_cost(trajectory_s) * FAST_2_TARGET_WEIGHT;
  double cost_closer_other_cars_any_lane = closer_other_cars_any_lane_cost(closets_car_any_lane) * CLOSER_OTHER_CARS_ANY_LANE_WEIGHT;
  double cost_closer_infront_car_same_lane = closer_infront_car_same_lane_cost(closets_car_any_lane) * CLOSER_INFRONT_CAR_SAME_LANE_WEIGHT;
  double cost_go_2_middle_lane = go_2_middle_lane_cost(trajectory_d) * GO_2_MIDDLE_LANE_WEIGHT;
  double cost_left_over_right = left_over_right_cost(trajectory_d) * LEFT_OVER_RIGHT_WEIGHT;

  //cout<<"cost_fast_2_target : "<< cost_fast_2_target<<endl;

  cost = cost_collision + cost_fast_2_target + cost_closer_other_cars_any_lane + cost_closer_infront_car_same_lane + cost_go_2_middle_lane + cost_left_over_right;
  
  return cost;
}

double collision_cost(double closest_vehicle_distance) {

  if (closest_vehicle_distance < (2.0 * VEHICLE_RADIUS)) return 1.0;
  else return 0.0;

}

double fast_2_target_cost(vector<double> trajectory_s) {
  // Low cost high velocity.
  double average_s_dot = 0.0;

  for(int i=1; i < trajectory_s.size(); i++){
    average_s_dot += ((trajectory_s[i] - trajectory_s[i-1] ) / 0.02);
    //cout<<"inside for : "<< (trajectory_s[i] - trajectory_s[i-1] )<<endl;
  }

  if ((trajectory_s.size()-1) > 0) average_s_dot /=  (trajectory_s.size()-1);
  //cout<<"average_s_dot : "<< average_s_dot<<endl;

  return logistic((MAX_VELOCITY - average_s_dot) / MAX_VELOCITY);
} 

double closer_other_cars_any_lane_cost(double closest_vehicle_distance) {

  return logistic(2 * VEHICLE_RADIUS / closest_vehicle_distance);

}

double closer_infront_car_same_lane_cost (double closest_vehicle_distance) {

  return logistic(2 * VEHICLE_RADIUS / closest_vehicle_distance);

}

double go_2_middle_lane_cost(vector<double> trajectory_d) {

  double middle_lane_dist = trajectory_d[trajectory_d.size()-1] - 6.0;
  return logistic( pow( middle_lane_dist , 2) );

}

double left_over_right_cost(vector<double> trajectory_d) {

  return logistic( trajectory_d[trajectory_d.size()-1] );

}
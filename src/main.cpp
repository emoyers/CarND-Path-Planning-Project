#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "macros.h"
#include "costs_and_JMT.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::map;
using std::set;
using std::cout;
using std::endl;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  Vehicle evo = Vehicle(initial_velocity, middle_lane);

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &evo]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //reference velocity to target in mph
  
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();

       
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          /*Size of the previous vector that contains the list of nest states*/
          int prev_size = previous_path_x.size();

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_s = car_s;
          double ref_d = car_d;
          double ref_s_dot = car_speed;
          double ref_d_dot = 0.0;
          double ref_s_dot_dot = 0.0;
          double ref_d_dot_dot = 0.0;
          double ref_yaw = deg2rad(car_yaw);

          if(prev_size<4){
            double prev_car_x = ref_x - cos(ref_yaw);
            double prev_car_y = ref_y - sin(ref_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(ref_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(ref_y);
          }
          else{
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            /*Calculate reference s and d*/
            vector<double> ref_s_d = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);
            
            ref_s = ref_s_d[0];
            ref_d = ref_s_d[1];

            //cout<<"ref_s :"<< ref_s<<endl;
            //cout<<"ref_d :"<< ref_d<<endl;

            /*Calculate reference s_dot and d_dot */
            double ref_x_prev_2 = previous_path_x[prev_size-3];
            double ref_y_prev_2 = previous_path_y[prev_size-3];
            ref_yaw = atan2(ref_y_prev-ref_y_prev_2, ref_x_prev-ref_x_prev_2);

            ref_s_d = getFrenet(ref_x_prev, ref_y_prev, ref_yaw, map_waypoints_x, map_waypoints_y);

            double ref_s_prev = ref_s_d[0];
            double ref_d_prev = ref_s_d[1];

            //cout<<"ref_s_prev :"<< ref_s_prev<<endl;
            //cout<<"ref_d_prev :"<< ref_d_prev<<endl;

            ref_s_dot = (ref_s - ref_s_prev)/0.02;
            ref_d_dot = (ref_d - ref_d_prev)/0.02;

            /*Calculate reference s_dot_dot and d_dot_dot */

            double ref_x_prev_3 = previous_path_x[prev_size-4];
            double ref_y_prev_3 = previous_path_y[prev_size-4];
            ref_yaw = atan2(ref_y_prev_2-ref_y_prev_3, ref_x_prev_2-ref_x_prev_3);

            ref_s_d = getFrenet(ref_x_prev_2, ref_y_prev_2, ref_yaw, map_waypoints_x, map_waypoints_y);

            double ref_s_prev_2 = ref_s_d[0];
            double ref_d_prev_2 = ref_s_d[1];

            //cout<<"ref_s_prev_2 :"<< ref_s_prev_2<<endl;
            //cout<<"ref_d_prev_2 :"<< ref_d_prev_2<<endl;

            double ref_s_dot_prev = (ref_s_prev - ref_s_prev_2)/0.02;
            double ref_d_dot_prev = (ref_d_prev - ref_d_prev_2)/0.02;

            ref_s_dot_dot = (ref_s_dot - ref_s_dot_prev)/0.02;
            ref_d_dot_dot = (ref_d_dot - ref_d_dot_prev)/0.02;


            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

          }

          evo.set_parameters_S_D(ref_s, ref_d, ref_s_dot, ref_d_dot, ref_s_dot_dot, ref_d_dot_dot);

          /*cout<<"evo.car_s :"<< evo.car_s<<endl;
          cout<<"evo.car_d :"<< evo.car_d<<endl;
          cout<<"evo.s_dot :"<< evo.s_dot<<endl;
          cout<<"evo.d_dot :"<< evo.d_dot<<endl;
          cout<<"evo.s_dot_dot :"<< evo.s_dot_dot<<endl;
          cout<<"evo.d_dot_dot :"<< evo.d_dot_dot<<endl;*/

          map<double, Vehicle> other_cars;
          map<double, vector<vector<double>> > other_cars_predictions;
          for (auto sf :  sensor_fusion)
          {
            Vehicle other_car;
            double other_car_speed = sqrt(pow((double)sf[3], 2) + pow((double)sf[4], 2));
            other_car.set_other_car_parameters( sf[1], sf[2], sf[5], sf[6], other_car_speed );
            other_cars_predictions[sf[0]] = other_car.other_cars_predict((int)(num_next_points_calulated-previous_path_x.size()), previous_path_x.size());
            other_cars[sf[0]] = other_car;
          }

          /*for(int i = 0; i<other_cars_predictions.find(sensor_fusion[0][0])->second[0].size();i++){
            cout<< "id 0: "<<
            i<<"  s: "<<other_cars_predictions.find(sensor_fusion[0][0])->second[0][i]<<
            ",  d: "<<other_cars_predictions.find(sensor_fusion[0][0])->second[1][i]<<
            endl;
          }*/

          bool other_car_front = false;
          bool other_car_right= false;
          bool other_car_left = false;
          for(auto it=other_cars_predictions.begin(); it!=other_cars_predictions.end();it++){
            double diff_s = fabs(evo.car_s - it->second[0][0]);
            double diff_d = -evo.car_d + it->second[1][0];
            //cout<<"diff_s: "<<diff_s<<"  diff_d: "<<diff_d<<endl;
            if(FREE_SPACE>diff_s){

              if (diff_d > 2 && diff_d < 6) other_car_right = true;
              else if (diff_d < -2 && diff_d > -6) other_car_left = true;
              else if (diff_d > -2 && diff_d < 2) other_car_front = true;
            }
          }

          if (other_car_right) cout << "CAR ON THE RIGHT!!!" << endl;
          if (other_car_left) cout << "CAR ON THE LEFT!!!" << endl;
          if (other_car_front) cout << "CAR JUST AHEAD!!!" << endl;
          if (!other_car_front & !other_car_left & !other_car_right ) cout << "CLEAR :)" << endl;
  
          if (other_car_front){
            evo.ref_velocity -= (1.0/2.24);
          }
          else if(evo.ref_velocity < (MAX_VELOCITY/2.24)){
            evo.ref_velocity += (1.0/2.24);
          }

          evo.set_possible_next_states(other_car_right, other_car_left);

          /*cout<<"posible states: ";
          for(auto it=evo.possible_next_states.begin(); it!=evo.possible_next_states.end();it++){
            cout<<*it<<", ";
          }
          cout<<endl;*/
          //cout<<"lane: "<<evo.actual_lane<<endl;
          double best_cost = 9999999;
          vector<vector<double>> best_target_s_and_d;
          vector<vector<double>> best_trajectory_s_and_d;

          for(string state: evo.possible_next_states){
            vector<vector<double>> target_s_and_d =evo.calculate_target_s_and_d(state, previous_path_x.size(), other_cars_predictions);
            /*
            cout<<state<<endl;
            cout<<"current_s: "<<evo.car_s<<" current_speed: "<<evo.s_dot<<" current_d :"<<evo.car_d<<endl;
            cout<<"next_s: "<<target_s_and_d[0][0]<<" next: "<<target_s_and_d[0][1]<<" next :"<<target_s_and_d[1][0]<<endl;
            */
            vector<vector<double>> trajectory_s_and_d =evo.get_trajectory(target_s_and_d, previous_path_x.size());
            /*
            cout<<state<<endl;
            cout<<"current_s: "<<evo.car_s<<" current_d :"<<evo.car_d<<endl;
            for(int f=0; f<trajectory_s_and_d[0].size();f++){
              cout<<"s_"<<f<<" : "<<trajectory_s_and_d[0][f]<<" d : "<<trajectory_s_and_d[1][f]<<endl;
            }
            */

            double cost_trajectory = calculate_trajectory_cost(trajectory_s_and_d, other_cars_predictions);
            //cout<<state<<" : "<<cost_trajectory<<endl;

            if ( cost_trajectory < best_cost ) {
                best_cost = cost_trajectory;
                best_target_s_and_d = target_s_and_d;
                best_trajectory_s_and_d = trajectory_s_and_d;
            }

          }

          /*cout<<"curret_s"<<evo.car_s<<endl;
          cout<<"current_d"<<evo.car_d<<endl;
          cout<<"best_target_s"<<best_target_s_and_d[0][0]<<endl;
          cout<<"best_target_d"<<best_target_s_and_d[1][0]<<endl;

          cout<<"-----------Trajectory-------------"<<endl;*/

          double best_target_s = best_target_s_and_d[0][0];
          double best_target_d = best_target_s_and_d[1][0];
          vector<double> target_xy = getXY(best_target_s, best_target_d, map_waypoints_s,map_waypoints_x,map_waypoints_y);


          vector<double> xy = getXY(evo.car_s+30,best_target_d, map_waypoints_s,map_waypoints_x,map_waypoints_y);
          ptsx.push_back(xy[0]);
          ptsy.push_back(xy[1]);

          xy = getXY(evo.car_s+60,best_target_d, map_waypoints_s,map_waypoints_x,map_waypoints_y);
          ptsx.push_back(xy[0]);
          ptsy.push_back(xy[1]);

          xy = getXY(evo.car_s+90,best_target_d, map_waypoints_s,map_waypoints_x,map_waypoints_y);
          ptsx.push_back(xy[0]);
          ptsy.push_back(xy[1]);

          /*change to local coordinates*/

          for (int i = 0; i < ptsx.size(); ++i)
          {
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }

          prev_size = previous_path_x.size();

          //cout<<"actual_x : "<<evo.car_x<<endl;
          //cout<<"actual_y : "<<evo.car_y<<endl;

          for (int i = 0; i < prev_size; ++i) {
            //cout<<"next_x"<<i<<" : "<<previous_path_x[i]<<endl;
            //cout<<"next_y"<<i<<" : "<<previous_path_y[i]<<endl;
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          tk::spline xy_spline;
          xy_spline.set_points(ptsx,ptsy); 

          double target_x = 30;
          double target_y = xy_spline(target_x);
          double target_dist =  sqrt((target_x*target_x)+(target_y*target_y));

          double x_add_on = 0;

          for(int i=0; i<num_next_points_calulated-prev_size ; i++){
            double N = (target_dist/(0.02*evo.ref_velocity));
            double x_point = x_add_on + (target_x)/N;
            double y_point = xy_spline(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref =  y_point;

            //rotate back to normal coordinates after rotating it earlier
            x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          //smooth driving 


          /*do not modify code below*/
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
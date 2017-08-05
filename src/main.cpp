#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"
#include "spline.h"
#include "helper.h"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  int car_state = 1;
  // 1 - drive_in_lane - drives at maximum speed or distance up to closest car in front
  // 2 - lane_change_needed - max speed obstructed in front, need to change lanes
  // 3 - changing_lane - change lane path clear, doing the change


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
  double dist_inc = 0.41; // max speed
  double targ_dist_inc;
  double targ_d;
  double pos_d;
  double pos_s;
  double prev_d;
  double prev_s;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  // correct spline calculation at the end of track
  map_waypoints_x.push_back(map_waypoints_x[0]);
  map_waypoints_y.push_back(map_waypoints_y[0]);
  map_waypoints_s.push_back(max_s+map_waypoints_s[0]);
  map_waypoints_dx.push_back(map_waypoints_dx[0]);
  map_waypoints_dy.push_back(map_waypoints_dy[0]);

  map_waypoints_x.push_back(map_waypoints_x[1]);
  map_waypoints_y.push_back(map_waypoints_y[1]);
  map_waypoints_s.push_back(max_s+map_waypoints_s[1]);
  map_waypoints_dx.push_back(map_waypoints_dx[1]);
  map_waypoints_dy.push_back(map_waypoints_dy[1]);


  tk::spline waypoints_x;
  waypoints_x.set_points(map_waypoints_s, map_waypoints_x);

  tk::spline waypoints_y;
  waypoints_y.set_points(map_waypoints_s, map_waypoints_y);

  tk::spline waypoints_dx;
  waypoints_dx.set_points(map_waypoints_s, map_waypoints_dx);

  tk::spline waypoints_dy;
  waypoints_dy.set_points(map_waypoints_s, map_waypoints_dy);

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &waypoints_x, &waypoints_y, &waypoints_dx, &waypoints_dy , &max_s, &car_state, &targ_dist_inc, &targ_d, &dist_inc, &pos_d, &pos_s, &prev_s, &prev_d](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    double max_s_change = 0.001;
    double max_d_change = 0.001;
    double min_dist_to_back_on_changelane = 10;
    double dist_front_empty_on_changelane = 50;
    double dist_front_speed_on_changelane = 30;

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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            // get closest car id's
            int num_cars = sensor_fusion.size();
            double min_dist = 99999;
            double min_sens_i;

            double min_dist_front_left = 99999;
            double min_dist_front_center = 99999;
            double min_dist_front_right = 99999;
            double min_dist_back_left = 99999;
            double min_dist_back_center = 99999;
            double min_dist_back_right = 99999;

            double min_sens_i_front_left;
            double min_sens_i_front_center;
            double min_sens_i_front_right;
            double min_sens_i_back_left;
            double min_sens_i_back_center;
            double min_sens_i_back_right;


            // finding closest cars in all 3 lanes, one in fron and one in back
            for (int i = 0; i < num_cars; ++i)
            {
              auto sens = sensor_fusion[i];
              double sens_id = sens[0];
              double sens_x = sens[1];
              double sens_y = sens[2];
              double sens_vel_x = sens[3];
              double sens_vel_y = sens[4];
              double sens_s = sens[5];
              double sens_d = sens[6];
              double sens_vel = sqrt(sens_vel_x*sens_vel_x + sens_vel_y*sens_vel_y);
              double sens_dist = distance(car_x,car_y,sens_x,sens_y);

              if ((sens_d > 4.0) and (sens_d<8.0)) // middle lane
              {
                if (sens_s > car_s) // car in front (center)
                {
                  if (sens_dist < min_dist_front_center) // find closest car in front
                  {
                    min_sens_i_front_center = i;
                    min_dist_front_center = sens_dist;
                  }
                }
                else // car behind (center)
                {
                  if (sens_dist < min_dist_back_center) // find closest car in front
                  {
                    min_sens_i_back_center = i;
                    min_dist_back_center = sens_dist;
                  }
                }
              }
              else
              {
                if (sens_d <= 4.0) // car in left lane
                {
                  if (sens_s > car_s) // car in front (left)
                  {
                    if (sens_dist < min_dist_front_left) // find closest car in front
                    {
                      min_sens_i_front_left = i;
                      min_dist_front_left = sens_dist;
                    }
                  }
                  else // car behind (left)
                  {
                    if (sens_dist < min_dist_back_left) // find closest car in front
                    {
                      min_sens_i_back_left = i;
                      min_dist_back_left = sens_dist;
                    }
                  }
                }
                else // right lane
                {
                  if (sens_s > car_s) // car in front (right)
                  {
                    if (sens_dist < min_dist_front_right) // find closest car in front
                    {
                      min_sens_i_front_right = i;
                      min_dist_front_right = sens_dist;
                    }
                  }
                  else // car behind (right)
                  {
                    if (sens_dist < min_dist_back_right) // find closest car in front
                    {
                      min_sens_i_back_right = i;
                      min_dist_back_right = sens_dist;
                    }
                  }
                }
              }
            }

            // state model
            // 1 - drive_in_lane - drives at maximum speed or distance up to closest car in front
            // 2 - lane_change_needed - max speed obstructed in front, need to change lanes
            // 3 - changing_lane - change lane path clear, doing the change

            //auto sens = sensor_fusion[min_sens_i];
            double sens_x;// = sens[1];
            double sens_y;// = sens[2];
            double sens_vel_x;// = sens[3];
            double sens_vel_y;// = sens[4];
            //double sens_s = sens[5];
            //double sens_d = sens[6];
            double in_front_vel;// = sqrt(sens_vel_x*sens_vel_x + sens_vel_y*sens_vel_y);
            double in_front_dist;// = distance(car_x,car_y,sens_x,sens_y);
            bool plan_lane_change = false;
            // calculating distance and velocity to car in front
            if (car_d <= 4)
            {
              sens_x = sensor_fusion[min_sens_i_front_left][1];
              sens_y = sensor_fusion[min_sens_i_front_left][2];
              sens_vel_x = sensor_fusion[min_sens_i_front_left][3];
              sens_vel_y = sensor_fusion[min_sens_i_front_left][4];
              if (car_state == 1 or car_state == 2)
              {
                targ_d = 2+0*4.0; // if car should be satying in lane, then set target to lane middle
              }
            }
            else
            {
              if (car_d >= 8)
              {
                sens_x = sensor_fusion[min_sens_i_front_right][1];
                sens_y = sensor_fusion[min_sens_i_front_right][2];
                sens_vel_x = sensor_fusion[min_sens_i_front_right][3];
                sens_vel_y = sensor_fusion[min_sens_i_front_right][4];
                if (car_state == 1 or car_state == 2)
                {
                  targ_d = 2+2*4.0;
                }
              }
              else
              {
                sens_x = sensor_fusion[min_sens_i_front_center][1];
                sens_y = sensor_fusion[min_sens_i_front_center][2];
                sens_vel_x = sensor_fusion[min_sens_i_front_center][3];
                sens_vel_y = sensor_fusion[min_sens_i_front_center][4];
                if (car_state == 1 or car_state == 2)
                {
                  targ_d = 2+1*4.0;
                }
              }
            }
            in_front_vel = sqrt(sens_vel_x*sens_vel_x + sens_vel_y*sens_vel_y);
            in_front_dist = distance(car_x,car_y,sens_x,sens_y);

            //vector<double> jmt_res;
            //int remaining_trajectory;

            if (car_state == 2) // need lane change
            {
              double pred_speed_to_car = in_front_dist/(50*dist_inc)+(in_front_vel/100-car_speed/100);
              targ_dist_inc = min(1.0, pred_speed_to_car)*dist_inc;
              bool can_change = false;

              double sens_x_l = sensor_fusion[min_sens_i_front_left][1];
              double sens_y_l = sensor_fusion[min_sens_i_front_left][2];
              double sens_vel_x_l = sensor_fusion[min_sens_i_front_left][3];
              double sens_vel_y_l = sensor_fusion[min_sens_i_front_left][4];
              double in_front_vel_l = sqrt(sens_vel_x_l*sens_vel_x_l + sens_vel_y_l*sens_vel_y_l);
              double in_front_dist_l = distance(car_x,car_y,sens_x_l,sens_y_l);
              double pred_speed_to_car_l = in_front_dist_l/(50*dist_inc)+(in_front_vel_l/100-car_speed/100);

              double sens_x_c = sensor_fusion[min_sens_i_front_center][1];
              double sens_y_c = sensor_fusion[min_sens_i_front_center][2];
              double sens_vel_y_c = sensor_fusion[min_sens_i_front_center][4];
              double sens_vel_x_c = sensor_fusion[min_sens_i_front_center][3];
              double in_front_vel_c = sqrt(sens_vel_x_c*sens_vel_x_c + sens_vel_y_c*sens_vel_y_c);
              double in_front_dist_c = distance(car_x,car_y,sens_x_c,sens_y_c);
              double pred_speed_to_car_c = in_front_dist_c/(50*dist_inc)+(in_front_vel_c/100-car_speed/100);

              double sens_x_r = sensor_fusion[min_sens_i_front_right][1];
              double sens_y_r = sensor_fusion[min_sens_i_front_right][2];
              double sens_vel_x_r = sensor_fusion[min_sens_i_front_right][3];
              double sens_vel_y_r = sensor_fusion[min_sens_i_front_right][4];
              double in_front_vel_r = sqrt(sens_vel_x_r*sens_vel_x_r + sens_vel_y_r*sens_vel_y_r);
              double in_front_dist_r = distance(car_x,car_y,sens_x_r,sens_y_r);
              double pred_speed_to_car_r = in_front_dist_r/(50*dist_inc)+(in_front_vel_r/100-car_speed/100);

              double sens_x_lr = sensor_fusion[min_sens_i_back_left][1];
              double sens_y_lr = sensor_fusion[min_sens_i_back_left][2];
              double in_back_dist_l = distance(car_x,car_y,sens_x_lr,sens_y_lr);

              double sens_x_cr = sensor_fusion[min_sens_i_back_center][1];
              double sens_y_cr = sensor_fusion[min_sens_i_back_center][2];
              double in_back_dist_c = distance(car_x,car_y,sens_x_cr,sens_y_cr);

              double sens_x_rr = sensor_fusion[min_sens_i_back_right][1];
              double sens_y_rr = sensor_fusion[min_sens_i_back_right][2];
              double in_back_dist_r = distance(car_x,car_y,sens_x_rr,sens_y_rr);

              cout << "Need to change lane, targ_d: " << targ_d << endl;

              if (targ_d < 4.0) // in left lane
              {
                cout << "in_front_vel_c: " << in_front_vel_c << endl;
                cout << "in_front_vel_l: " << in_front_vel_l << endl;
                cout << "in_front_dist_c: " << in_front_dist_c << endl;
                cout << "in_back_dist_c: " << in_back_dist_c << endl;
                if ((in_front_dist_c > dist_front_empty_on_changelane) and
                    (in_back_dist_c > min_dist_to_back_on_changelane))
                {
                  cout << "Changing on empty center" << endl;
                  targ_d = 2+1*4.0;
                  can_change = true;
                }
                else
                {
                  if ((in_front_dist_c > dist_front_speed_on_changelane) and
                  (in_front_vel_c > in_front_vel_l) and
                  (in_back_dist_c > min_dist_to_back_on_changelane))
                  {
                    cout << "Changing on faster center" << endl;
                    targ_d = 2+1*4.0;
                    can_change = true;
                  }
                }
                /*
                //if (pred_speed_to_car_c > pred_speed_to_car_l)
                if (((in_front_vel_c > in_front_vel_l) and
                                    (in_front_dist_c > dist_front_speed_on_changelane)) or
                                    (in_front_dist_c > dist_front_empty_on_changelane))
                {
                  cout << "Considering change to CENTER: " << endl;
                  if (in_back_dist_c > min_dist_to_back_on_changelane)
                  {
                    targ_d = 2+1*4.0;
                    can_change = true;
                    cout << "--> changing" << endl;
                  }
                }
                */
              }
              else
              {
                if (targ_d > 8.0)
                {
                  cout << "in_front_vel_c: " << in_front_vel_c << endl;
                  cout << "in_front_vel_r: " << in_front_vel_r << endl;
                  cout << "in_front_dist_c: " << in_front_dist_c << endl;
                  cout << "in_back_dist_c: " << in_back_dist_c << endl;
                  if ((in_front_dist_c > dist_front_empty_on_changelane) and
                      (in_back_dist_c > min_dist_to_back_on_changelane))
                  {
                    cout << "Changing on empty center" << endl;
                    targ_d = 2+1*4.0;
                    can_change = true;
                  }
                  else
                  {
                    if ((in_front_dist_c > dist_front_speed_on_changelane) and
                    (in_front_vel_c > in_front_vel_r) and
                    (in_back_dist_c > min_dist_to_back_on_changelane))
                    {
                      cout << "Changing on faster center" << endl;
                      targ_d = 2+1*4.0;
                      can_change = true;
                    }
                  }
                  /*
                  //if (pred_speed_to_car_c > pred_speed_to_car_r)
                  if (((in_front_vel_c > in_front_vel_r) and
                                    (in_front_dist_c > dist_front_speed_on_changelane)) or
                                    (in_front_dist_c > dist_front_empty_on_changelane))
                  {
                    cout << "Considering change to CENTER: " << endl;
                    if (in_back_dist_c > min_dist_to_back_on_changelane)
                    {
                      targ_d = 2+1*4.0;
                      can_change = true;
                      cout << "--> changing" << endl;
                    }
                  }
                  */
                }
                else
                {
                  // car is in center lane and need decision where to change
                  bool empty_left = false;
                  bool empty_right = false;
                  bool longest_empty_left = false;
                  bool safe_change_left = false;
                  bool safe_change_right = false;

                  if (in_front_dist_l > dist_front_empty_on_changelane)
                  {
                    empty_left = true;
                    cout << "empty_left" << endl;
                  }
                  if (in_front_dist_r > dist_front_empty_on_changelane)
                  {
                    empty_right = true;
                    cout << "empty_right" << endl;
                  }
                  if (in_front_dist_l > in_front_dist_r)
                  {
                    longest_empty_left = true;
                    cout << "longest_empty_left" << endl;
                  }
                  if ((in_back_dist_l > min_dist_to_back_on_changelane) and
                      (in_front_dist_l > dist_front_speed_on_changelane))
                  {
                    safe_change_left = true;
                    cout << "safe_change_left" << endl;
                  }
                  if ((in_back_dist_r > min_dist_to_back_on_changelane) and
                      (in_front_dist_r > dist_front_speed_on_changelane))
                  {
                    safe_change_right = true;
                    cout << "safe_change_right" << endl;
                  }
                  // if both lanes are empty and safe to change then choose longest
                  bool processed = false;
                  if (empty_left and empty_right and safe_change_left and safe_change_right)
                  {
                    if (longest_empty_left)
                    {
                      targ_d = 2+0*4.0;
                      can_change = true;
                      cout << "--> changing on left empty longest" << endl;
                    }
                    else
                    {
                      targ_d = 2+2*4.0;
                      can_change = true;
                      cout << "--> changing on right empty longest" << endl;
                    }
                    processed = true;
                  }
                  // changing left on empty
                  if (!processed and empty_left and safe_change_left)
                  {
                    targ_d = 2+0*4.0;
                    can_change = true;
                    processed = true;
                    cout << "--> changing on left empty" << endl;
                  }
                  // changing right on empty
                  if (!processed and empty_right and safe_change_right)
                  {
                    targ_d = 2+2*4.0;
                    can_change = true;
                    processed = true;
                    cout << "--> changing on right empty" << endl;
                  }

                  // case when not empty lane
                  if (!processed)
                  {
                    if (in_front_vel_l > in_front_vel_r)
                    {
                      if (((in_front_vel_l > in_front_vel_c) and
                                      (in_front_dist_l > dist_front_speed_on_changelane)) or
                                      (in_front_dist_l > dist_front_empty_on_changelane))
                      {
                        cout << "Considering change to LEFT: " << endl;
                        if (in_back_dist_l > min_dist_to_back_on_changelane)
                        {
                          targ_d = 2+0*4.0;
                          can_change = true;
                          cout << "--> changing" << endl;
                        }
                      }
                    }
                    else
                    {
                      if (((in_front_vel_r > in_front_vel_c) and
                                        (in_front_dist_r > dist_front_speed_on_changelane)) or
                                        (in_front_dist_r > dist_front_empty_on_changelane))
                      {
                        cout << "Considering change to RIGHT: " << endl;
                        if (in_back_dist_r > min_dist_to_back_on_changelane)
                        {
                          targ_d = 2+2*4.0;
                          can_change = true;
                          cout << "--> changing" << endl;
                        }
                      }
                    }
                  }
                  /*
                  if (pred_speed_to_car_l > pred_speed_to_car_r)
                  {
                    if (pred_speed_to_car_l > pred_speed_to_car_c)
                    {
                      targ_d = 2+0*4.0;
                      can_change = true;
                    }
                  }
                  else
                  {
                    if (pred_speed_to_car_r > pred_speed_to_car_c)
                    {
                      targ_d = 2+2*4.0;
                      can_change = true;
                    }
                  }
                  */
                }
              }

              if (can_change)
              {
                // generate trajectory for d to start following
                //jmt_res = JMT({car_d, 0.0, 0.0}, {targ_d, 0.0, 0.0}, 50);
                //remaining_trajectory = 50;
                //cout << "jmt: " << jmt_res[0] << endl;
                plan_lane_change = true;
                car_state = 3;
              }
            }
            else
            {
              if (car_state == 3) // changing lane
              {
                //double pred_speed_to_car = in_front_dist/(50*dist_inc)+(in_front_vel/100-car_speed/100);
                //targ_dist_inc = min(1.0, pred_speed_to_car)*dist_inc;
                // When lane change finished, change the vehicle state
                if (abs(car_d - targ_d)<0.1)
                {
                  car_state = 1;
                  //remaining_trajectory = 0;
                }
              }
              else // default state, driving at max or follow speed
              {
                double pred_speed_to_car = in_front_dist/(50*dist_inc)+(in_front_vel/100-car_speed/100);
                targ_dist_inc = min(1.0, pred_speed_to_car)*dist_inc;
                if (pred_speed_to_car < 1.0) // need lane change
                {
                  car_state = 2;
                }
              }
            }

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            double pos_x;
            double pos_y;
            //double pos_s;
            //double pos_d;
            double angle;
            double speed;

            int path_size = previous_path_x.size();

            // Copy old path
            for(int i = 0; i < path_size; i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // Initialize path to current position if no path
            if(path_size == 0)
            {
              pos_x = car_x;
              pos_y = car_y;
              angle = deg2rad(car_yaw);
              pos_s = car_s;
              pos_d = car_d;
              end_path_d = car_d;
              speed = 0.0;
              prev_s = car_s;
              prev_d = car_d;
              //cout << "Speed 0 " << endl;
            }
            else // Calculate angle and get position from previous data
            {
              pos_x = previous_path_x[path_size-1];
              pos_y = previous_path_y[path_size-1];

              double pos_x2 = previous_path_x[path_size-2];
              double pos_y2 = previous_path_y[path_size-2];
              angle = atan2(pos_y-pos_y2,pos_x-pos_x2);

              double delta_x = pos_x2-pos_x;
              double delta_y = pos_y2-pos_y;
              speed = distance(pos_x, pos_y, pos_x2, pos_y2) / 0.02;

              //pos_s = end_path_s;
              //pos_d = end_path_d;
              //cout << "Speed2: " << speed << endl;
            }
            // we have targ_d and targ_dist_inc to the target in optimistic case
            // TODO: setting path to not exceed parameters

            //cout << "car state: " << car_state;
            //cout << "\t tar s: " << targ_dist_inc;
            //cout << "\t tar d: " << targ_d;
            //cout << "\t car d: " << car_d << endl;
            double d_smoothing_steps = 50; // TODO: Calculate parameter based
            double delta = (targ_d - end_path_d) / d_smoothing_steps;

            // Setup path waypoints
            double path_point_x;
            double path_point_y;
            double path_point_dx;
            double path_point_dy;


            // We have values
            // end_path_s
            // end_path_d
            // targ_dist_inc
            // targ_d
            //cout << "car state: " << car_state;
            //cout << "\t tar s: " << targ_dist_inc;
            //cout << "\t end s: " << end_path_s;
            //cout << "\t tar d: " << targ_d;
            //cout << "\t end d: " << end_path_d;
            //cout << "\t car d: " << car_d << endl;

            //cout << "Before 50 loop: Pos_s " << pos_s;
            //cout << "\t Pos_d " << pos_d;
            //cout << "\t Delta " << delta << endl;
            /*
            double jmt0;
            double jmt1;
            double jmt2;
            double jmt3;
            double jmt4;
            double jmt5;
            if (remaining_trajectory>=1)
            {
              jmt_res[0] = jmt0;
              jmt_res[1] = jmt1;
              jmt_res[2] = jmt2;
              jmt_res[3] = jmt3;
              jmt_res[4] = jmt4;
              jmt_res[5] = jmt5;
            }
            */

            // Create path for lane change
            if (plan_lane_change)
            {
              // take last position and last position -10
              // take target position and target position +10
              // create spline
              vector<double> line_change_s;
              vector<double> line_change_d;
              line_change_s.push_back(pos_s-4);
              line_change_d.push_back(prev_d);
              line_change_s.push_back(pos_s-3);
              line_change_d.push_back(prev_d);
              line_change_s.push_back(pos_s-2);
              line_change_d.push_back(prev_d);
              line_change_s.push_back(pos_s-1);
              line_change_d.push_back(prev_d);
              line_change_s.push_back(pos_s);
              line_change_d.push_back(prev_d);

              line_change_s.push_back(pos_s+50);
              line_change_d.push_back(targ_d);
              line_change_s.push_back(pos_s+54);
              line_change_d.push_back(targ_d);

              tk::spline path_d;
              path_d.set_points(line_change_s, line_change_d);


              for(int i = 0; i < 50; i++)
              {
                double delta_prev_s = pos_s - prev_s;
                //double delta_prev_d = pos_d - prev_d;
                prev_s = pos_s;
                //prev_d = pos_d;
                pos_s += min(targ_dist_inc, (delta_prev_s*(1+0.005)+0.001));
                pos_s = fmod(pos_s, max_s);
                pos_d = path_d(pos_s);
                path_point_x = waypoints_x(pos_s);
                path_point_y = waypoints_y(pos_s);
                path_point_dx = waypoints_dx(pos_s);
                path_point_dy =  waypoints_dy(pos_s);
                //double lane_d = pp.d;

                pos_x = path_point_x + path_point_dx * pos_d;
                pos_y = path_point_y + path_point_dy * pos_d;

                next_x_vals.push_back(pos_x);
                next_y_vals.push_back(pos_y);
              }
              plan_lane_change = false;
            }



            // rewrite for loop for lane change
            if (path_size<50)
            {
              for(int i = 0; i < 50-path_size; i++)
              {
                //next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
                //next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
                //pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
                //pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
                //pos_s += dist_inc;
                //cout << "I: " << i;
                double delta_prev_s = pos_s - prev_s;
                double delta_prev_d = pos_d - prev_d;
                prev_s = pos_s;
                prev_d = pos_d;
                //cout << "\t DS: " << delta_prev_s;
                //cout << "\t DP: " << delta_prev_d;
                pos_s += min(targ_dist_inc, (delta_prev_s*(1+0.005)+0.001));
                pos_s = fmod(pos_s, max_s);


                if (delta>0.0)
                {
                  pos_d += min(delta, 0.05);
                }
                else
                {
                  pos_d += max(delta, -0.05);
                }

                /*
                if (remaining_trajectory<1)
                {
                  if (delta>0.0)
                  {
                    pos_d += min(delta, 0.05);
                  }
                  else
                  {
                    pos_d += max(delta, -0.05);
                  }
                }
                else
                {
                  double t = 50-remaining_trajectory;
                  pos_d = jmt0 + jmt1*t + jmt2*t*t + jmt3*t*t*t + jmt4*t*t*t*t + jmt5*t*t*t*t*t;
                  remaining_trajectory -= 1;
                  cout << "Coef pos_d: " << pos_d << endl;
                }
                */
                //cout << "\t Pos S: " << pos_s;
                //cout << "\t Pos D: " << pos_d << endl;

                path_point_x = waypoints_x(pos_s);
                path_point_y = waypoints_y(pos_s);
                path_point_dx = waypoints_dx(pos_s);
                path_point_dy =  waypoints_dy(pos_s);
                //double lane_d = pp.d;

                pos_x = path_point_x + path_point_dx * pos_d;
                pos_y = path_point_y + path_point_dy * pos_d;

                next_x_vals.push_back(pos_x);
                next_y_vals.push_back(pos_y);
              } // end for loop
            } // end if (path_size<50)

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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

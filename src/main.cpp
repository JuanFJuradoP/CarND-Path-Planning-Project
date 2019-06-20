/*

Code Information:

	Developer: Juan Jurado - JJ.
	Phone:  / +1 (513) 909 4704 / +57 (313) 247 4186.
	Mail: juanjuradop@gmail.com / jj@kiwicampus.com.
  LinkedIn: https://www.linkedin.com/in/juanfjuradop/

Description: 
    Self-Driving Car Nanodegree Program.
    Part 2: Localization, Path Planning, Control, and System Integration
        Project # 2: Highway Driving.
    Objective: The goal of this project is to build a path planner that creates smooth, 
              safe trajectories for the car to follow. The highway track has other vehicles, 
              all going different speeds, but approximately obeying the 50 MPH speed limit.

Tested on: 
    Ubuntu 16.04.

Project Specification:

*Compilation*
(CHECK)  The code compiles correctly - Code must compile without errors with cmake and make.
(CHECK)  The code compiles correctly - Given that we've made CMakeLists.txt as general as possible,
    it's recommend that you do not change it unless you can guarantee that your changes will 
    still compile on any platform.

Valid Trajectories:

(CHECK)  The car is able to drive at least 4.32 miles without incident - The top right screen of the
    simulator shows the current/best miles driven without incident. Incidents include exceeding 
    acceleration/jerk/speed, collision, and driving outside of the lanes. Each incident case is 
    also listed below in more detail.
(CHECK)  The car drives according to the speed limit - The car doesn't drive faster than the speed
    limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.
(CHECK)  Max Acceleration and Jerk are not Exceeded - The car does not exceed a total acceleration 
    of 10 m/s^2 and a jerk of 10 m/s^3.
(CHECK)  Car does not have collisions - The car must not come into contact with any of the other 
    cars on the road.
(CHECK)  The car stays in its lane, except for the time between changing lanes - The car doesn't 
    spend more than a 3 second length out side the lane lanes during changing lanes, and every
    other time the car stays inside one of the 3 lanes on the right hand side of the road.
(CHECK)  The car is able to change lanes - The car is able to smoothly change lanes when it makes 
    sense to do so, such as when behind a slower moving car and an adjacent lane is clear of 
    other traffic.
  
Reflection:

(Readme Attached) There is a reflection on how to generate paths - The code model for generating paths
    is described in detail. This can be part of the README or a separate doc labeled "Model 
    Documentation".

*/



/*
*****************************************************************************************************
*****************************************************************************************************
*****************************************************************************************************

COMENTS:

- I used the file 'helpers.h' to declare the useful functions that I will use in the project, also 
  add more documentation to each of the functions.
- Video QA / UDACITY:
  https://www.youtube.com/watch?v=7sI3VHFPP0w
- The only repository that I reviewed to complete the project is that of Professor Aaron Brown.
  https://github.com/awbrown90/CarND-Path-Planning-Project

  I understood every line of code used and even generated additional code with documentation to give
  continuity to the pipeline. Thank you for your advice. I would like to learn and improve my skills.

*****************************************************************************************************
*****************************************************************************************************
*****************************************************************************************************
*/

// ==================================================================================================
// IMPORT SOME LIBRARIES AND FILES
// ==================================================================================================
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <vector>
#include <iomanip>

#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// Lane 0 - Left lane
// Lane 1 - Medium Lane
// Lane 2 - Right Lane

// The highway has 3 lanes. The left, central and right lane. Usually the left lane is used to overtake
// the slower cars and the right lane is used to go at a low speed, therefore, it will be indicated 
// that the car always looks for the slowest lane. (Right Lane).

int lane = 2; // Safe lane is the right one.

int lane_change_wp = 0;

using json = nlohmann::json;

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
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          
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

          //  The maximum speed that the car can reach is the double double variable MAX_SPEED = 49.0, 
          //  that value was chosen because it allows me to have a safety margin of 1mph.
          double MAX_SPEED = 49.0; // Max speed allowed by the car [mph].
          double SPEED_RATE = 0.224; // Speed rate (increase and decrease)
          vector<double> x_points;
          vector<double> y_points;

          // Update some variables 
          int previous_path_size = previous_path_x.size();
          int next_waypoint = -1; // Next waypoint setted up as -1. Indicates there is no path assigned.
          double xPos_reference = car_x;  // Copy variable from car_x to capture the current value.
          double yPos_reference = car_y;  // Copy variable from car_y to capture the current value.
          double yaw_reference = deg2rad(car_yaw); // Convert deg to rad and capture de current value. 

          // The end point of the previous path is the new reference.
          // Update the ref state as the end point previous path  
          if(previous_path_size < 2){
          	next_waypoint = NextWaypoint(xPos_reference, yPos_reference, yaw_reference, map_waypoints_x,map_waypoints_y,map_waypoints_dx,map_waypoints_dy);
          }else{
            // Define some delta variables
            double yPos_reference_prev, xPos_reference_prev, xDelta_reference, yDelta_reference;

            yPos_reference_prev = previous_path_y[previous_path_size-2];
            xPos_reference_prev = previous_path_x[previous_path_size-2];

            xPos_reference = previous_path_x[previous_path_size-1];
				    yPos_reference = previous_path_y[previous_path_size-1];

            // Delta assignment variables
				    xDelta_reference = xPos_reference-xPos_reference_prev;
            yDelta_reference = yPos_reference-yPos_reference_prev;

            // atan2 calculate
				    yaw_reference = atan2(yDelta_reference,xDelta_reference);

            car_s = end_path_s;

            car_speed = (sqrt( ( xDelta_reference*xDelta_reference ) + ( yDelta_reference*yDelta_reference ))/.02)*2.237;
            next_waypoint = NextWaypoint(xPos_reference,yPos_reference,yaw_reference,map_waypoints_x,map_waypoints_y,map_waypoints_dx,map_waypoints_dy);
				    
          }

          double closestDist_s = 2000; 

          // Bool variable that indicates if the car can change the lane.
          bool lane_changer = false;
          //  True: Indicates I can change the lane
          //  False: Indicates I can't change the lane

          // Debug console information
          std::cout << "Car info:  " << "s = " << car_s << std::setw(11) << "d = " << car_d << std::setw(11) << "Speed = " << car_speed << std::endl;
          
          for(int i = 0; i < sensor_fusion.size(); i++){

            //  Check if the car is in my lane
            float frenet_car_pos = sensor_fusion[i][6];

            // Just to recap each lane is about 4 meters
            if(frenet_car_pos < (2+4*lane+2) && frenet_car_pos > (2+4*lane-2)){

              // capture car's speed
              double x_speed = sensor_fusion[i][3];
              double y_speed = sensor_fusion[i][4];
              double speed_measurement = speed_measurement_f(x_speed,y_speed);

              // Frenet coordinates calculation. check_car_s
              double check_car_s = sensor_fusion[i][5];
              
              check_car_s+=((double)previous_path_size*.02*speed_measurement);

              //I Check if it is 20 meters to target (car)
              if((check_car_s > car_s) && ((check_car_s-car_s) < 20) && ((check_car_s-car_s) < closestDist_s )){
                closestDist_s = (check_car_s - car_s);
                if((check_car_s-car_s) > 20){
                  //match that cars speed
                  MAX_SPEED = speed_measurement*2.237; //2.237 number to convert to m/s
                  lane_changer = true;
                }else{
                  // Speed down
                  MAX_SPEED = speed_measurement*2.237-5;
                  lane_changer = true;
                }
              }
            }
          }

          // Situation: If the car is behing to another cart.
          // Action: Try to change the lane to go ahead. (Evnetually)
          if(lane_changer == true && ( (next_waypoint-lane_change_wp) % map_waypoints_x.size() > 2) ){
            
            bool changed_lanes = false;

            // Passing to the left lane

            if(lane != 0 && !changed_lanes){

              bool lane_safe = true; // It is a safe lane (true)

              for(int i = 0; i < sensor_fusion.size(); i++){

                // Car position is in the left lane.
                float frenet_car_pos = sensor_fusion[i][6];

                // Just to recap each lane is about 4 meters
                if(frenet_car_pos < (2+4*(lane-1)+2) && frenet_car_pos > (2+4*(lane-1)-2)){

                  // capture car's speed
                  double x_speed = sensor_fusion[i][3];
                  double y_speed = sensor_fusion[i][4];
                  double speed_measurement = speed_measurement_f(x_speed,y_speed);

                  // Frenet coordinates calculation. check_car_s
                  double check_car_s = sensor_fusion[i][5];
                  check_car_s+=((double)previous_path_size*.02*speed_measurement);

                  double dist_s = check_car_s-car_s;
                  if(dist_s < 20 && dist_s > -20){
                    lane_safe = false;
                  }
                }
              }
              if(lane_safe){
                changed_lanes = true;
                lane -= 1;
                lane_change_wp = next_waypoint;
              }
            }
            // Passing to the right lane

            if(lane != 2 && !changed_lanes){

              bool lane_safe = true;

              for(int i = 0; i < sensor_fusion.size(); i++){

                // Check if the car is in my lane - Right lane
                float frenet_car_pos = sensor_fusion[i][6];

                // Just to recap each lane is about 4 meters
                if(frenet_car_pos < (2+4*(lane+1)+2) && frenet_car_pos > (2+4*(lane+1)-2)){
                  
                  // capture car's speed
                  double x_speed = sensor_fusion[i][3];
                  double y_speed = sensor_fusion[i][4];

                  double speed_measurement = speed_measurement_f(x_speed,y_speed);

                  // Frenet coordinates calculation. check_car_s
                  double check_car_s = sensor_fusion[i][5];
                  check_car_s+=((double)previous_path_size*.02*speed_measurement);

                  double dist_s = check_car_s-car_s;
                  if(dist_s < 20 && dist_s > -20){
                    lane_safe = false;
                  }
                }
              }
              if(lane_safe){
                changed_lanes = true;
                lane += 1;
                lane_change_wp = next_waypoint;
              }
            }
          }

          if(previous_path_size < 2){

            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            x_points.push_back(prev_car_x);
            x_points.push_back(car_x);

            y_points.push_back(prev_car_y);
            y_points.push_back(car_y);

          }else{

            x_points.push_back(previous_path_x[previous_path_size-2]);
            x_points.push_back(previous_path_x[previous_path_size-1]);

            y_points.push_back(previous_path_y[previous_path_size-2]);
            y_points.push_back(previous_path_y[previous_path_size-1]);
          }

          vector<double> waypoint_0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> waypoint_1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> waypoint_2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          x_points.push_back(waypoint_0[0]);
          x_points.push_back(waypoint_1[0]);
          x_points.push_back(waypoint_2[0]);

          y_points.push_back(waypoint_0[1]);
          y_points.push_back(waypoint_1[1]);
          y_points.push_back(waypoint_2[1]);

          
          for (int i = 0; i < x_points.size(); i++ ){
          
            double shift_x = x_points[i]-xPos_reference;
            double shift_y = y_points[i]-yPos_reference;

            x_points[i] = (shift_x *cos(0-yaw_reference)-shift_y*sin(0-yaw_reference));
            y_points[i] = (shift_x *sin(0-yaw_reference)+shift_y*cos(0-yaw_reference));
          }

          // https://kluge.in-chemnitz.de/opensource/spline/
          // Create a spline called 's'
          tk::spline s;

          // Set (x,y) points to spline
          s.set_points(x_points,y_points);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for(int i = 0; i < previous_path_x.size(); i++){

            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);

          }

          double x_pos_spline = 60.0; // points that are part of the sampling of the spline - 
          // If we have more points the smoother is the spline

          // Calculate the spline
          double y_pos_spline = s(x_pos_spline);

          // Basically, this approach is a triangle. The poin A is the car and the point B is the target.
          // I calculate the hypotenuse of this triangle.
          double spline_distance = sqrt((x_pos_spline)*(x_pos_spline)+(y_pos_spline)*(y_pos_spline));
          
          double x_addition = 0;

          for (int i = 1; i <= 50-previous_path_x.size(); i++){
            
            // The function update_speed() is responsible for updating the value of the current speed.
            // It compares the current speed of the car with the maximum value of the speed allowed.
            // Our goal is to go to the maximum speed allowed (MAX_SPEED) without create accidents.

            car_speed = update_speed(car_speed, MAX_SPEED, SPEED_RATE);

            // 2.24 is the factor number to convert mph to m/s
            double N = (spline_distance/(.02*car_speed/2.24));

            double xp = x_addition+(x_pos_spline)/N;
            double yp = s(xp);

            x_addition = xp;

            double x_ref = xp;
            double y_ref = yp;

            // The previous calculations are in local coordinates, you need to transform to global coordinates
            xp = (x_ref *cos(yaw_reference)-y_ref*sin(yaw_reference));
            yp = (x_ref *sin(yaw_reference)+y_ref*cos(yaw_reference));

            xp += xPos_reference;
            yp += yPos_reference;

            next_x_vals.push_back(xp);
            next_y_vals.push_back(yp);
          }

          json msgJson;
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
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <fstream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int lane = 1;
double target_speed = 0.0;
double safe_dist = 20.0;
double target_spacing = 40.0;
int path_size = 50;

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
    // define lane and reference velocity slightly below speed limit
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
          
          // the size of the previous path
          int prev_size = previous_path_x.size();

          /* Collision avoidance */
          if(prev_size > 0){
            car_s = end_path_s;
          }

          bool car_ahead = false;
          bool car_left = false;
          bool car_right = false;

          // Defining variables for finding information about the car directly ahead on the same lane
          double dist_car_ahead = 10000;
          int id_car_ahead = -1;
          double speed_car_ahead = 0;

          // Use sensor fusion to find reference velocity to move at by looping through all the cars on the road
          // sensor_fusion vector [ id, x, y, vx, vy, s, d]
          for(int i = 0; i < sensor_fusion.size(); i++){
            // find out if another car is in the same lane as our ego car
            int car_id = sensor_fusion[i][0];
            float d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(pow(vx,2)+pow(vy,2));  // speed magn.

            double check_car_s = sensor_fusion[i][5];

            check_car_s += (double)prev_size * 0.02 * check_speed; // prediction: projecting the cars position into the future by using previous points
            
            double dist2othercar = check_car_s - car_s;
            int lane_other_car = d/4; 
            // todo: check if this is needed
            if(lane_other_car < 0 || lane_other_car > 2){
              continue;
            }
          
            // setting flags
            if(lane == lane_other_car){  // if car is in same lane
              car_ahead |= check_car_s > car_s && check_car_s - car_s < safe_dist;
              // Getting information about car in front of us
              if(check_car_s > car_s && dist2othercar > 0){
                id_car_ahead = car_id;
                speed_car_ahead = check_speed * 2.24; //m/sec to mph
                dist_car_ahead = dist2othercar;
              }
            }
            else if(lane-lane_other_car == 1){  // if car is on the left lane of us
              car_left |= car_s - safe_dist < check_car_s && car_s + safe_dist > check_car_s;
            }
            else if(lane-lane_other_car == -1){ // if car is on the right lane of us
              car_right |= car_s - safe_dist < check_car_s && car_s + safe_dist > check_car_s;
            }
          }

          // take actions
          double speed_diff = 0;
          if(car_ahead){
            if(!car_left && lane > 0){  //no car on left lane and we are on middle lane or right lane
              lane--;
            }
            else if(!car_right && lane!=2){  //no car on right lane and we are on middle lane or left lane
              lane++;
            }
            else {
              speed_diff -= 0.224;  // slow down
            }
          }
          // set actions for free driving (aka no car in front) -> keep right as possible
          else{
            // todo: maybe can be simplified as well?
            if(lane != 2){ // if we are not on the center lane.
              if((lane == 0 && !car_right)){
                lane = 1; // Back to center.
              }
              else if((lane==1 && !car_right)){
                lane = 2;  // Back to right
              }
            }
            if(target_speed < 49.5){
              speed_diff += 0.224;
            }
          }
          
          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // Generate a smooth trajectory by creating a couple of widely spaced waypoints which are e.g. 30m apart and then fit spline through those points
          vector <double> ptsx;
          vector <double> ptsy;

          // Create reference x, y, yaw points; either where car is at or where previous path ends
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = car_yaw;
          double ref_speed = -1.0;

          // if previous path almost empty, use state of car
          if(prev_size < 2){
            // Generate two points to make path that's tangent to car's state
            double prev_car_x = car_x - cos(car_yaw);  
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);  //first point
            ptsx.push_back(car_x);  //second point

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);

            // add reference speed
            ref_speed = car_speed;

          }

          // if we can build upon previous path
          else{
            // set reference state as previous path endpoints
            ref_x = previous_path_x[prev_size-1];  // last point
            ref_y = previous_path_y[prev_size-1];

            double prev_ref_x = previous_path_x[prev_size-2]; // penultimate point
            double prev_ref_y = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);

            ptsx.push_back(prev_ref_x);
            ptsx.push_back(ref_x);

            ptsy.push_back(prev_ref_y);
            ptsy.push_back(ref_y);

            ref_speed = distance(prev_ref_x, prev_ref_y, ref_x, ref_y) / 0.02;
          }
          std::cout<<"Ref speed = "<<ref_speed<<std::endl;

          // create evenly spaced points e.g. 30m apart starting from the reference points (can be defined using variable int apart = 30)
          vector<double> next_wp0 = getXY(car_s+target_spacing*1, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+target_spacing*2, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+target_spacing*3, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // now there are five points which define the trajectory of the next cycle

          // transformation into the car's coordinates / point of view
          for(int i = 0; i<ptsx.size(); i++){
            // translation
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            // ... plus rotation
            ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
            ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
          }

          // now fitting points by creating spline
          tk::spline s;
          s.set_points(ptsx, ptsy);

          // Build new path by starting with previous path
          for(int i = 0; i<previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Spacing points of generated spline in order to keep desired speed
          double target_x = target_spacing;
          double target_y = s(target_x);  // what is y for given x according to spline function
          double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

          double x_add_on = 0.0;

          // Add points of Spline to new path to fill up remaining points of the path (previous path + new path generated by spline)
          for(int i = 0; i < path_size-previous_path_x.size(); i++){  // assuming the path always consists of path_size points
            target_speed += speed_diff;
            if(target_speed > 49.5){
              target_speed = 49.5;
            }
            else if(target_speed < 0.224){
              target_speed = 0.224;
            }
            double N = target_dist/(0.02*target_speed/2.24); // Number of points for splitting up the trajectory along the target distance; converting from mph to m/sec, evaluating new point every 20 ms
            double x_point = x_add_on + target_x / N;  // next x point
            double y_point = s(x_point);  // evaluating y point along the spline

            x_add_on = x_point;
            double x_ref = x_point;
            double y_ref = y_point;

            // transform back to global coordinate from car coordinates
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);


          }


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

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
    /*
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
  */

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
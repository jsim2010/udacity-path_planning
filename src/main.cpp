#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

			static double lane = 1;
			static double desired_lane = 1;
			static double speed_limit = 50.0;
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

					int prev_size = previous_path_x.size();
				  double max_speed = speed_limit;
					bool shift_left = false;
					bool shift_right = false;

					// Start with previous points that haven't been used.
          vector<double> next_x_vals;
          vector<double> next_y_vals;

					for (int i = 0; i < previous_path_x.size(); i++) {
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}

					if (prev_size == 0) {
						end_path_s = car_s;
					}

					// Collect data about other cars going in same direction
					// Stores [s, end_s, speed] about each car.
					vector<vector<vector<double>>> other_cars;

					for (int i = 0; i < 3; i++) {
						vector<vector<double>> lane;
						other_cars.push_back(lane);
					}

					for (int i = 0; i < sensor_fusion.size(); i++) {
						double other_lane = ((double)sensor_fusion[i][6]) / 4.0;

						if (other_lane >= 0) {
							double vx = sensor_fusion[i][3];
							double vy = sensor_fusion[i][4];
							double v = sqrt(vx * vx + vy * vy);
							double other_s = sensor_fusion[i][5];
							double other_end_s = other_s + ((double)prev_size) * 0.02 * v;
							vector<double> other_car;

							other_car.push_back(other_s);
							other_car.push_back(other_end_s);
							other_car.push_back(v * 2.24);
							other_cars[other_lane].push_back(other_car);
						}
					}

					// Check current lane for slower cars in front.
					for (int i = 0; i < other_cars[lane].size(); i++) {
						double other_end_s = other_cars[lane][i][1];
						double other_speed = other_cars[lane][i][2];

						if ((other_end_s > end_path_s) && ((other_end_s - end_path_s) < 20) && (other_speed < max_speed)) {
							// Slow down to match the car in front.
							max_speed = other_speed;
						}
					}

					// If desired to switch lanes, find which ones are possible.
					if (max_speed < speed_limit) {
						// If lane exists to left.
						if (lane > 0) {
							shift_left = true;
							int possible_lane = lane - 1;

							for (int i = 0; i < other_cars[possible_lane].size(); i++) {
								double other_s = other_cars[possible_lane][i][0];
								double other_end_s = other_cars[possible_lane][i][1];

								if (other_s > (car_s - 10) && other_end_s < (end_path_s + 20)) {
									shift_left = false;
									break;
								}
							}
						}

						// If lane exists to right.
						if (lane < 2) {
							shift_right = true;
							int possible_lane = lane + 1;

							for (int i = 0; i < other_cars[possible_lane].size(); i++) {
								double other_s = other_cars[possible_lane][i][0];
								double other_end_s = other_cars[possible_lane][i][1];

								if (other_s > (car_s - 10) && other_end_s < (end_path_s + 20)) {
									shift_right = false;
									break;
								}
							}
						}
					}

					// Change lanes.
					if (shift_right) {
						lane += 1;
					} else if (shift_left) {
						lane -= 1;
					}

					// Build a path to use spline on.
					vector<double> x_vals;
					vector<double> y_vals;
					vector<double> current_point;
					vector<double> prev_point;
					double start_yaw;
					double speed;

					if (prev_size > 0) {
						current_point.push_back(previous_path_x[prev_size - 1]);
						current_point.push_back(previous_path_y[prev_size - 1]);
						prev_point.push_back(previous_path_x[prev_size - 2]);
						prev_point.push_back(previous_path_y[prev_size - 2]);
						double x_dist = current_point[0] - prev_point[0];
						double y_dist = current_point[1] - prev_point[1];

						start_yaw = atan2(y_dist, x_dist);
						speed = sqrt(x_dist * x_dist + y_dist * y_dist) * 2.24 / 0.02;
					} else {
						current_point.push_back(car_x);
						current_point.push_back(car_y);
						prev_point.push_back(car_x - cos(car_yaw));
						prev_point.push_back(car_y - sin(car_yaw));
						start_yaw = car_yaw;
						speed = car_speed;
					}

					x_vals.push_back(prev_point[0]);
					y_vals.push_back(prev_point[1]);
					x_vals.push_back(current_point[0]);
					y_vals.push_back(current_point[1]);

					for (int i = 0; i < 3; i++) {
						vector<double> point = getXY(car_s + (50.0 * (i + 1)), (lane * 4) + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
						x_vals.push_back(point[0]);
						y_vals.push_back(point[1]);
					}

					// Transform path so that start is (0, 0) and yaw is 0.
					for (int i = 0; i < x_vals.size(); i++) {
						double shift_x = x_vals[i] - current_point[0];
						double shift_y = y_vals[i] - current_point[1];

						x_vals[i] = shift_x * cos(0 - start_yaw) - shift_y * sin(0 - start_yaw);
						y_vals[i] = shift_x * sin(0 - start_yaw) + shift_y * cos(0 - start_yaw);
					}

					// Setup spline
					tk::spline s;

					s.set_points(x_vals, y_vals);

					/// Fill up remaining points.
					double target_x = 30.0;
					double target_y = s(target_x);
					double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
					double spline_x = 0.0;

					for (int i = 0; i < 50 - prev_size; i++) {
						// Adjust speed towards slightly under max_speed.
						if (speed < (max_speed - 1)) {
							speed += 0.25;
						} else if (speed > (max_speed - 0.5)) {
							speed -= 0.25;
						}

						double N = target_dist / (0.02 * speed / 2.24);
						spline_x += target_x / N;
						double spline_y = s(spline_x);

						double x = spline_x * cos(start_yaw) - spline_y * sin(start_yaw) + current_point[0];
						double y = spline_x * sin(start_yaw) + spline_y * cos(start_yaw) + current_point[1];

						next_x_vals.push_back(x);
						next_y_vals.push_back(y);
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

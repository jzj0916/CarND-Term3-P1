#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "vehicle.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

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

  // the lane to drive
  int lane = 1;
  // the reference speed in mph !
  double ref_v = 0.0;

  const double kTargetV = 49.5;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_v, &kTargetV](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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
          	vector< vector<double> > sensor_fusion = j[1]["sensor_fusion"];

            int prev_size = previous_path_x.size();

            // set current Frenet s pose to the prev calculated path end s,
            // i.e in the future from current standpoint as the end point
            // on the previously calculated path is not reached yet.
          	if (prev_size > 0) {
              car_s = end_path_s;
            }

            bool prox_warn = false;

            for (int i = 0; i < sensor_fusion.size(); i++) {
              // only consider cars in ego lane
              double obj_d = sensor_fusion[i][6]; // d values of cars
              if (obj_d < (2 + 4 * lane + 2) && obj_d > (2 + 4 * lane - 2) ) {
                // collect all the coordinates of the object
                double obj_x = sensor_fusion[i][3];
                double obj_y = sensor_fusion[i][4];
                double obj_speed = std::sqrt( obj_x * obj_x + obj_y * obj_y);
                double obj_s = sensor_fusion[i][5];

                // project object s value out, as using previous path
                // 0.02 = 20 milliseconds
                // idea is to predict where the object will be the next frame
                obj_s += ((double)prev_size * 0.02 * obj_speed);
                //check if object is in front of ego vehicle and
                // distance is less than the safe distance
                if ( (obj_s > car_s) && ((obj_s - car_s) < 30) ) {
                  // too close to object in front, proximity warning
                  prox_warn = true;


                  // TODO:

                  // CHANGE LANE LOGIC


                  // END




                } // if obj_s
              } // if obj_d
            }// for


            // handle speed based on objects ahead

            // TODO: if KL match the speed of the vehicle ahead (safe speed for the lane) ??
            // Or instead of matching the speed, maintain safe distance!!
            // while PLCL PLCR:
            // match position and speed of "gap" to change lane

            if (prox_warn) {
              ref_v -= 0.224; // decelerate with 5 ms^2
            } else if (ref_v < kTargetV) {
              ref_v += 0.224; // accelerate with 5 ms^2
            }

            // create list of widely spaced (x,y) waypoints
            vector<double> pts_x;
            vector<double> pts_y;

            //reference x, y, yaw states
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // if previous size is empty, use the car as start reference
            if (prev_size < 2) {
              // two points that make a tangent to the car
              double prev_car_x = car_x - std::cos(car_yaw);
              double prev_car_y = car_y - std::sin(car_yaw);

              pts_x.push_back(prev_car_x);
              pts_x.push_back(car_x);

              pts_y.push_back(prev_car_y);
              pts_y.push_back(car_y);

            } else {
              // redefine reference state as previous path end point
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = std::atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              // use 2 points that make path tangent to prev path's endpoint
              pts_x.push_back(ref_x_prev);
              pts_x.push_back(ref_x);

              pts_y.push_back(ref_y_prev);
              pts_y.push_back(ref_y);

            }// if-else

            // In Frenet, add evenly spaced (by 30 meters) points ahead
            // of starting reference
            vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            pts_x.push_back(next_wp0[0]);
            pts_x.push_back(next_wp1[0]);
            pts_x.push_back(next_wp2[0]);

            pts_y.push_back(next_wp0[1]);
            pts_y.push_back(next_wp1[1]);
            pts_y.push_back(next_wp2[1]);

            for (int i = 0; i < pts_x.size(); i++) {
              // transform coordinates to car coordinates (car reference frame)
              // shift car reference angle to 0 degrees
              double shift_x = pts_x[i] - ref_x;
              double shift_y = pts_y[i] - ref_y;
              double rot_angle = 0.0 - ref_yaw;
              // rotate
              pts_x[i] = (shift_x * std::cos(rot_angle) - shift_y * std::sin(rot_angle));
              pts_y[i] = (shift_x * std::sin(rot_angle) + shift_y * std::cos(rot_angle));

            } // for

            // std::cout << " Size of pts_x: " << pts_x.size() << " Size of pts_y: " << pts_y.size() << std::endl;

            // create spline
            tk::spline spl;
            // set points to the spline
            spl.set_points(pts_x, pts_y);

            // define x, y points to use for the planner

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            // start with all the previous points from the last epoch
            for (int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            } // for

            // calculate how to break up spline points, so that
            // we maintain the reference velocity
            double target_x = 30.0;
            double target_y = spl(target_x);
            double target_dist = std::sqrt((target_x * target_x) + (target_y * target_y) );

            double x_add_on = 0.0;

            // fill up rest of path after filling it with prev points
            // here output is always 50 points
            for (int i = 0; i < 50 - previous_path_x.size(); i ++) {
              // divide by 2.24 to get from mph to meters per second;
              double N = (target_dist / (0.02 * ref_v / 2.24 ) );
              double x_point = x_add_on + (target_x) / N;
              double y_point = spl(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // rotate back to global coordinates after rotating it earlier
              x_point = (x_ref * std::cos(ref_yaw) - y_ref * std::sin(ref_yaw) );
              y_point = (x_ref * std::sin(ref_yaw) + y_ref * std::cos(ref_yaw) );

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);

            } // for


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

            // double dist_incr = 0.3;

            // double next_d = 6;
            // double next_s;
            // for (int i = 0; i < 50; i++){
            //   next_s = car_s + (i + 1) * dist_incr;
            //   vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            //   next_x_vals.push_back(x[0]);
            //   next_y_vals.push_back(x[1]);


            // }// for



            json msgJson;
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
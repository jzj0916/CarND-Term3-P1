# Behaviour Planning for Autonomous Driving 

Self-Driving Car Engineer Nanodegree Program
   
### Simulator

The project is based on the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals

In this project the goal is to implement a path planner in C++ to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. 

The simulator provides the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The aim is for the car to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, executing safe and smooth lane changes, avoiding collisions with other vehicles. 
The car should stay within the lanes and avoid exceeding a maximum acceleration of 10 m/s^2 and a maximum jerk of 10 m/s^3.

### Demo

Please click on the link below to see a demo of the vehicle driving around on the simulator track.

[Demo video](https://youtu.be/8ER7H1pEDnQ)


### Path Generation Model

The algorithm uses the sensor data to determine lane availability. Based on this the ego vehicle can have one of three states:
Keep Lane, Change Lane Left, or Change Lane Right.

The algorithm also sets the velocity of the vehicle based on the lane availability and the presense of a vehicle ahead.
If there is a vehicle ahead of the ego vehicle and the distance is less than a set safe distance, and no lane change is possible, the velocity is set to the velocity of the vehicle ahead until a suitable lane change can be executed.
The ego vehicle accelerates or slows down accordingly to match the traffic's velocity or follow the optimal set velocity (just below the speed limit of 50 MPH).

Once the state of the vehicle changes from Keep Lane to Change Lane, a smooth trajectory is calculated using the last points from a previously calculated path together with a spline curve based on three knots (points) at 30, 60 and 90 meters ahead of the vehicle. Then 50 points are taken from the entire curve and these points are fed back into the simulator, whereby the spacing of the points determines the velocity of the vehicle. 


### Discussion

The vehicle is able to complete a lap without any collisions and violations of the acceleration and jerk limits. The Best Distance Without Incident was tested at 27 miles. Setting the safe distance parameters to lower values, e.g. 10 meters for safe distance ahead and only 5 meters for safe distance behind the vehicle, allows for a more aggressive driving and exploitation of gaps in the traffic, but with a higher risk of a collision. One possible improvement is to have the safety distance determined by the speed of the traffic ahead, whereby in slower moving traffic the safe distance can be decreased.

The addition of a Bayesian classifier to predict the behaviour of other vehicles can help to optimise the lane changes. The current algorithm, faced with a choice of left or right lane change, selects left lane change without testing if a right lane change is more optimal based on traffic further ahead.

The addition of a Finite State Machine to manage the different states and the transitions, along with a suitable set of cost functions can also improve the behaviour, as the current algorithm cannot track a gap in a neighboring lane, match its speed and make use of it.


### Simulator Data

#### The map

Map of the highway is in data/highway_map.txt

Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 




## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.



## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).






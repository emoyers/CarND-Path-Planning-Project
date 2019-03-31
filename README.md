[image0]: ./images/screen.png "original chessboard"
# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Code Explanation

The code consists of 8 files, 3 source files and 5 header files:
* main.cpp
* vehicle.cpp
* costs_and_JMT.cpp
* vehicle.h
* macros.h
* costs_and_JMT.h
* helpers.h
* spline.h

In the `main.cpp` is the logic that is divided in:
* Initialization form line `115` to `201`
* Getting other car trajectories form line `210` to `255`
* Calculating best trajectory for our car from line `263` to `302`, this include calculating trajectories for all the possible states (keep in the same lane, change right and change left)
* Signal conditioning for next path form line `311` to `370`

In `vehicle.cpp` is the definition of `Vehicle class` use to store the information of our car and other cars. This class contains the following methods:

* **set_parameters** : Initialization of our car
* **set_other_car_parameters** : Initialization of the other cars
* **set_parameters_S_D** : setting s and d related parameters
* **other_cars_predict** : get the predict trajectory of the other cars, using sensor fusin data
* **set_possible_next_states** : get the possible state of our car, base on the other car trajectories
* **calculate_target_s_and_d** : calculate the target s and d base on possible states
* **get_trajectory** : return the trajectory based on the calculated target s and d

The file `costs_and_JMT.cpp` have the JMT algorithm and the cost fuctions that wer use to decide the best trajectory. The function that calculate the cost of every trajectory is `calculate_trajectory_cost` located from line `102` to `125`.
The cost the are considered within the previous function are:

* **collision_cost** : penalize the collisions
* **fast_2_target_cost** : rewards faster speeds
* **closer_other_cars_any_lane_cost** : penalize being close to other vehicles
* **closer_infront_car_same_lane_cost** : penalize being close to the vehicle in front
* **go_2_middle_lane_cost** : rewards been in the center lane, to have more options for the next decision
* **left_over_right_cost** : prefer a left lane change than a right lane change

I didn't use jerk cost or acceleration cost because I use JMT and spline for the final points calculation in `main.cpp`

`vehicle.h` and `costs_and_JMT.h` contain the prototypes of `vehicle.cpp` and `costs_and_JMT.cpp` respectively.

`macros.h` have all the constant used within the project.

`helpers.h` already included in the original repository.

`spline.h` is used to generate the next values for the next loop base on the calculated best trajectory, to avoid reaching great jerk and acceleration and make the movement of the car more smoothly.

Next the image is a prove of reaching the 4.32 miles without an incident and following the constrains mentioned in the rubric.

![alt text][image0]
**Simulator screen shot**

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

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

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

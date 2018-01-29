# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   


### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.  The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Approach
The path planner is divide in **two main stages**, the change lane stage and avoid colision and the trajectory generation.


## Change Lanes and avoid collision
In this stage the path planner checks if there is car ahead, if there is, the target velocity is set to velocity of fist vehicle ahead. Also is checked if safe to change lanes.

The code below illustrate the Change Lanes and avoid collision stage

``` c++
max_vel = 49.5;
vector<bool> safe_lanes = {true, true, true};
bool change = false;
double closeset_car_s = 10000;
// loop trough cars
for (int i = 0; i < sensor_fusion.size(); i++) {
  float d = sensor_fusion[i][6];
    // loop in lanes
  for (int l = 0; l < 3; l++) {
    // checks if is in lane l
    if (d < (2 + 4 * l + 1.75) && d > (2 + 4 * l - 1.75)) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = sensor_fusion[i][5];
      // car position at time of the last previous point
      check_car_s += prev_size * dt * check_speed;
      // my lane and have car ahead in less than 30m
      if (lane == l && check_car_s > car_s && check_car_s - car_s < 30 &&
        closeset_car_s >= check_car_s) {
      // reduce  target velocity if car ahead is less than 30 m from us
        max_vel = check_speed - 1;
      closeset_car_s = check_car_s;


      safe_lanes[l] = false;

      //change lane if there is a car ahead 40m or less
    } else if (lane == l && check_car_s > car_s && check_car_s - car_s < 40) {
      change = true;
    }
     // cannot chenge to lane l if there is a car ahead with 30m or less
    else if (check_car_s > car_s && abs(check_car_s - car_s) < 30) {

      safe_lanes[l] = false;
      // cannot chenge to lane l if there is a car behind with 10m or less
    } else if (check_car_s < car_s && car_s - check_car_s < 10) {
      safe_lanes[l] = false;
...
if (ref_vel < max_vel) {
   // if is in lane 1
  if (lane == 1 && change) {
   // is safe to go to lane 0
    if (safe_lanes[0]) {
      lane = 0;
      max_vel = 49.5;
      change = false;
    } else if (safe_lanes[2]) {
      lane = 2;
      max_vel = 49.5;
                           
      change = false;
    }
  }

  if (lane == 0 && change) {
    if (safe_lanes[1]) {
      lane = 1;
      max_vel = 49.5;
      change = false;
    }
  }
...
```

## Trajectory Generation
This stage generate the trajectory base on reference velocity ( velocity the car should achieve defined in early stage), and at maximum 50 points ahead in future each point will be consumed by the simulator in an interval of 0.2 seconds. 

The trajectory is controlled by a spline with previous control points not processed by the simulator and following 3 points space by 30 meters in Frenet coordinate system. Even when the lane is changed in the *Change Lanes and avoid collision stage* the spline interpolation garantes a smooth trajectory.

``` c++
//center of lane 
double t = (2 + 4 * lane);
auto next_wp0 = get_xy(car_s + 30, t);
auto next_wp1 = get_xy(car_s + 60, t);
auto next_wp2 = get_xy(car_s + 90, t);
```

with the spline configured the next points are interpolated using the code below:

``` c++
// target coordinate to measure incremental points using the spline
  double target_x = 30.;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  double x_add_on = 0;
  // discretetize the points ahead in cars coordinate , (50 - previous_path_x.size()) chunks
  for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
   // adjust velocity in using an acceleration of 5m/s2
    if (ref_vel > max_vel) {
        ref_vel -= 0.223;
    } else {
        ref_vel += 0.223;
    }

    // percentage of displacement in direction to target
    double N = ((dt * ref_vel / 2.24) /target_dist); 
    // percentage in x is proportional to the target direction
    double x_point = x_add_on + (target_x * N); 
    // y interpolated by spline
    double y_point = s(x_point);
                            
    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;
    // rotate and shifts back to world coordinate
    std::tie(x_point, y_point) = rotate(x_ref, y_ref, ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);

    next_y_vals.push_back(y_point);

}
```


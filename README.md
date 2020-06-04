# **Project7 : Path Planning Project** 

### Objectives 

To design a path planner that will be able to generate smooth, safe paths for the car to travel along a 3 lane highway with traffic. 

---

### Overview
The proposed path planner uses the highway map data, telemetry data and sensor fusion information to successfully plan safe trajectories while passing the slower moving traffic, avoids hitting other cars and keeps inside its lane when necessary. The vehicle has driven successfully without any incidents for more than the 4.32 miles requested in the project rubric. 

![Image1](https://github.com/saikrishnachada/CarND-Path-Planning-Project/blob/master/result.PNG)  
---

### Reflection

The procedure discussed in the Project Q&A video was helpful in clearly understanding how to move the car from its initial position, stick to the lane using frenet coordinates, speed control while approaching a leading vehicle and pass the vehicle by changing to the left lane by using spline method. However, these are the basic things discussed to successfully start the project and I have improvised them further to be able to function effectively throughout the highway. Additionally, to improve the readability of the code, it is nicely commented. 

##### Improvised code in main.cpp

In the code lines (106-125), we first initialize the variables car_leader, car_left and car_right to be false, assuming that there is no traffic. However, we check in the next steps using the sensor fusion data, running through each traffic car and checking in which lane the traffic car is. 

In the code lines (134-144), we check if there is a safe distance between the other traffic objects in either left/right lane both from and behind to be able to perform a successful lane change. We replace the initial car_leader, car_left and car_right guess with the new boolean true/false. True meaning it is not safe to shift to that respective lane and false meas that the traffic cars are far away so that we are ready for a lane change. 

In the code lines (147-167), using the previous results we perform a lane change when it is safe to change the lanes, we decelerate when it is not safe to switch to the other lane and there is a leader vehicle traveling slow in our lane. Moreover, we intend to reach the maximum road speed limit by slowly accelerating and avoiding jerks as well.

In the code lines (254-259), we are trying to increase or decrease the reference speed in a way to avoid jerks while starting from initial position and also are trying to not cross the maximum road speed limit. 

Other than the above mentioned changes the code remains the same as discussed in the Project Q&A video. 

### Results
* The code complies without any errors with cmake and make
* The car is able to drive longer than mentioned 4.32 miles requirement without any incident. 
* The car doesn't drive faster than the speed limit (50 mph). Moreover, the car only slows down if obstructed by the traffic. 
* The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
* The car perfectly avoids collision with the other cars on the road.
* The car does safe lane changes with perfect trajectories. In case of unsafe conditions, the car decelerates and does the speed control if it is obstructed by a leader vehicle. 

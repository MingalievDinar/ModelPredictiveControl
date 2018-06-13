# CarND-Controls-MPC
PROJECT SPECIFICATION
---

[//]: # (Image References)

[image1]: ./pic/state.png "State"
[image2]: ./pic/model_eq.png "Model equations"
[image3]: ./pic/cte1.png "Cross track error"
[image4]: ./pic/cte2.png "Cross track error"
[image5]: ./pic/psi1.png "Orientation error"
[image6]: ./pic/psi2.png "Orientation error"
[image7]: ./pic/onTrack.png "On track"

## Compilation

1. Compile: `cmake .. && make`
2. Run it: `./mpc`.

## Implementation

### The Model
Kinematic model is used for MPC implementation. It is a simplification of dynamic models that ignore tire forces, gravity, and mass.
This simplification reduces the accuracy of the models, but it also makes them more tractable.

The state is a vector of position of the car, its velocity and angle: [x, y, v, psi].

![alt text][image1]

Actuators - [δ, a] - steering angle and throttle.

Kinematic equations used for prediciting the next state:

![alt text][image2]

Lf measures the distance between the front of the vehicle and its center of gravity. The larger the vehicle, the slower the turn rate.

We incliuded cte (cross-track error) and eψ (psi error) in addition to 4 element state vecor.
The new state is [x,y,ψ,v,cte,eψ].
The error between the center of the road and the vehicle's position as the cross track error

![alt text][image3]

cte can be expressed as the difference between the line and the current vehicle position y. The reference line is polinom, calculated at the position of the car. (Polynom coefficients are obtained from waypoints provided).

![alt text][image4]

The orientation error

![alt text][image5]

eψ is the desired orientation subtracted from the current orientation:

![alt text][image6]

ψ desired can be calculated as the tangential angle of the polynomial f evaluated at x.

### Timestep Length and Elapsed Duration (N & dt)

N = 10 and dt = 0.1 were shoosen after many other experiments with speed and cost function weights.
These parameters showed the most stable results. It was suggested that T = N*dt is less than 2.5 sec.
I have tried N = 20 and different dt in {0.1, 0.5, 1} and the car was running not stable.
By reducing N = 10 and playing with dt and weights It was decided so end up with dt = 0.1.

![alt text][image7]

T < 1 sec is not enough to make proper turns when angles are very steep.

### Polynomial Fitting and MPC Preprocessing

The waypoints and the vehicle state were preprocessed.
* waypoints coordinate system was moved to the vehicle state (x, y) --> so we can work in the vehicle coordinate system.
* in addition, it was turned to minus psi angel in that way as the vehicle moves horisontally.
So, we can see that x, y and psi are all zero. It helps for polynomial fitting becase now it is look like a function (for each x we have only one y).
Shiftint the waypoint's coordinat system to the car coordinate system and rotating them to psi angle is done in the `main.cpp` file in lines 108-118

### Model Predictive Control with Latency
The latency of 100 milliseconds was taken into account in file `main.cpp` in lines 101-105. The kinematic equations descibed above were used to shift the vehicle state to a new state in 100 ml seconds forward.



> # Reflection

# Model

## State
- There are two coordinate systems used in this project:
    - World(Map): This is used by the simulator. This is what the server returns.
    - Car: This is used by the car's model

There are six components:
- The current x position of the car (px)
- The current y position of the car (py)
- The heading(driving direction) of the car related to postitive x axis (psi)
- The magnitude of the speed of the car (v)
- The cross-track error (CTE)
- The error of the driving direction (epsi)

## Actuation

There are two components:
- steering: In the model this is in [-25 degree, 25 degree]. It is rescaled to [-1 radian,1 radian] before sending to the simulator
- throttle: How much the car will accelerate in Unity unit (m) / (unit timestep (s)) ^ 2. It is in [-1, 1]

## Transition Function
- A.k.a. Update Equations
- This is based on the kinematic model in the lecture. 
- Because the car's kinematic model uses the car's coordinate system, the car is always at (0,0) with a driving direction (psi) at 0 radian. A more simplified model can be used. This model will be described later
- This is also used to predict the car's state in a future state when the actuation is delayed ( by 100ms in this project)
- This model assumes the car will drive in constant velocity between consecutive timesteps, and the speed will be changing linearly between consecutive timesteps. In addition, this model assumes the car has a constant turning rate between consecutive timesteps. The CTE and epsi are calculated from the error between model and actual position
- $x_{t+1} = x_{t} + v_{t} * dt$
- $y_{t+1} = x_{t} + v_{t} * dt$ 
- $v_{t+1} = v_{t} + a_{t} * dt$
- $psi_{t+1} = psi_{t} + v_{t} * steer_{t} / Lf * dt$
- $cte_{t+1} = (f_{t} - y_{t}) + (v_{t} + sin(epsi_{t}) * dt)$
- $epsi_{t+1} = psi_{t} - tan^{-1}(f'_{t}) + v_{t} * steer_{t} / Lf * dt$

# Timestep Length and Elapsed Duration (N & dt)

- The polynomial is fitted to all the waypoints provided by the server, no matter whether the waypoint is in front of the car or on the back of the car
- Therefore, the polynomials are circular, it is nessesary to set an upper limit of timestep length in order to eliminate the polynomial segments that move to the back of the car.
- However, the car must be able to look ahead far enough to make a good judgement of the curvature of the road, so N should also be large anough.
- The choice of dt is sensitive to the actuation latency. If the latency is short, dt should be small, if the latency is large, dt should be large.
- When dt is large, the car is less reactive to turns. When dt is small, the car is more reactive to turns
- Previous value tried:
    - N = 50, dt = 0.03. The fitted polynomial is way to the sides of the road that cannot see on the simulator, which causes the car drive to the sides when turning.
    - N = 10, dt = 0.05. The fitted polynomial is too short to see on the simulator. The car has a very limited foresight of the road, and does not respond to any road curvature changes.
    - N = 25, dt = 0.05. This value works when the car has a 100ms actuation latency, but it causes the car unresponsive to significant turns when there is no actuation latency.
    - N = 25, dt = 0.03. This value works when the car does not have an actuation latency, but it causes the car overreact to turns when there is a 100ms latency.

# Polynomial Fitting and MPC Preprocessing

## Polynomial Fitting

- Because there are curves in the simulator and in real life, using linear model ( order 1 polynomial) does not work well
- Using quadratic model (order 2 polynomial) could fit most curves well, and it could deal with consecutive curves turning in the same direction. However, it cannot fit consecutive curves turning in different directions.
- Therefore, a cubic model (order 3 polynomial) is needed. 
- Using models over order 3 could work better if there are more data, but there are only 6 waypoints available from the simulator. Therefore, using a more complex model would cause overfitting


## Preprocessing

- Before running the MPC solver, I transformed the waypoints from map's coordinate system ( what the server sends to the program) to car's coordindate system, where the current position of the car is (0,0), and the current heading of the car is 0 radian. 
- Transforming the waypoints simplifies the model, which is helpful for programming and debugging. This process also normalize the data by making them around 0, which makes the prediction more accurate.
    - $f_{t,1}$ means the coefficient of the 1st order term of the polynomial 
    - $x_{t} = 0, y_{t} = 0, psi_{t} = 0$
    - $x_{t+1} = x_{t} + v_{t} * dt$
    - $y_{t+1} = 0$  ( The car will always be moving at the current direction until it receives a new command and starts a new session of MPC calculation )
    - $v_{t+1} = v_{t} + a_{t} * dt$
    - $psi_{t+1} = v_{t} * steer_{t} / Lf * dt$
    - $cte_{t+1} = (f_{t} - y_{t}) + (v_{t} + sin(-tan^{-1}(f_{t,1})) * dt)$
    - $epsi_{t+1} = - tan^{-1}(f_{t,1}) + v_{t} * steer_{t} / Lf * dt$
- I also tested the MPC solver using both map's coordinate system and car's coordinate system. The model using the map's coordinate system could only predict closer points correctly, and the predicted curve curves to the back of the car for farther points. This causes the car to U-turn where it should drive straight ahead. However, the model using the car's coordinate system solves this problem.
- In order to combat the actuation latency, I added another preprocessing step, see the section below.

    

# Model Predictive Control with Latency

- Before I put the car's state into the MPC solver, I used the transformed and simplified kinetic model stated in the "Preprocessing" section (right above) to predict the state of the car at 100ms later. I then pass the predicted state to the MPC solver.
- I also increased dt from 0.03 to 0.05. See "Timestep Length and Elapsed Duration (N & dt)" section for more detail.
- The MPC controller can drive the car safely at 50mph if there were no latency. However, it can only drive the car at 40mph with latency. If the car is driven at 50mph, it can still be on the track if there is no big turns, but it will fall off the track when in big turns.


```python

```

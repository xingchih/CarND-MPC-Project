# MPC Project Write-Up

## 1. Model
### States
The model consists of six states. Four of them are the vechicle states: the vechicle's X and Y position (px and py), orientation (psi) and velocity (v). The other two are the error terms: cross-track error (cte), i.e., deviation in vehicle Y direction, and orientation error (epsi). 

### 1.1. Actuators
Two actuator inputs are the steering angle (delta) and the acceleration (a), i.e., the inputs a driver would provide to the vehicle.

### 1.2. Update Equations 
At time step k, we can update our model for k+1 with the system states and actuator inputs using following equations:

  px_k+1 =  px_k + v_k * cos(psi_k) * dt
  py_k+1 =  py_k + v_k * sin(psi_k) * dt
 psi_k+1 = psi_k + v_k/Lf * delta_k * dt
   v_k+1 =   v_k + a_k * dt
 cte_k+1 = f(x_k) - y_k + ( v_k * sin(epsi_k) * dt )
epsi_k+1 = psi_k - psi_des_k + ( v_k/Lf * delta_k * dt )

where dt is the time difference between timestep k+1 and k, Lf is the distance from front wheels to center of gravity of the vehicle, f(x_k) is the y value from the fitted polynomial at x_k, psi_des_k is the desired orientation at timestep k (psi_des_k = atan(coeffs[1] + 2*coeffs[2]*x_k + 3*coeffs[3]*x_k*x_k).

### 1.3. Timestep Length and Elapsed Duration (N & dt)
The final values chosen for N and dt are N = 10, and dt = 0.1. Various values have been experimented, such as N = [5, 10, 15, 20 ,25] and dt = [0.1, 0.25, 0.5, 0.75, 1]. Increasing N will slow down the optimization solver and large dt will predict too far in advance in the future which often will cause large discrepancy between desried and actual trajectory. With N = 10 and dt = 0.1, we are looking at 1 second with adequate time needed by the optimization solver. 

## 2. Polynomial Fitting and MPC Preprocessing
The actuator inputs are and should be in the vehicle coordinate frame. The waypoints provided from the simulator are in absolute map coordinate frame and need to be transformed into the vehicle coordinate frame (vechicle forward is X and lateral left is Y). This transform also improves polynomial fitting, because most of the time the vehicle heading is pretty close to its X-axis and we wouldn't get large spikes in Y. After transforming the waypoints into vechicle coordinate frame, we fit the waypoints with a third order polynomial, compute cte and epsi, as well as the initial states for optimization (all in vechicle coordinate frame). For exmpale, the initial states px_0 = 0, py_0 = 0, and psi_0 = 0, if no latency is considered. The initial states and polynomial coefficients are then passed into MPC solver to obtain the actuator inputs. The returned values are the steering angle (delta), which we scale down by (deg2rad(25)*Lf), and acceleration (a). Both are passed back to the simulator.

## 3. Latency
A time delay of 100 ms is injected when passing back the actuator inputs back to the simulator. To compensate for this latency, we modify the initial states with following state update equations with dt = 0.1 (100 ms): 

px  = px + v * cos(psi) * dt;
py  = py + v * sin(psi) * dt;
psi = psi + v/Lf * mpc.delta/Lf * dt;
v   = v + mpc.a * dt;

where mpc.delta and mpc.a are actuator inputs from previous cycle. This predicts where the vehicle will be and its states in 100 ms. With these new states px, py, psi, and v, we can perform the same processing as stated in previous section: transform waypoints into vehicle coordinate frame, fit polynomial, calculate cte and epsi, and perform MPC optimization. The vehicle now can run at a faster speed because the MPC compensates the latency and enables a more stable behavior.


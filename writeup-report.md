### The Model

The MPC model includes the **state**, **actuators** and update **equations**.

##### State
* `px`   - position-x
* `py`   - position-y
* `psi`  - angle offset from center
* `v`    - velocity
* `cte`  - cross track error
* `epsi` - error in psi 

##### Actuators
* `delta` - steer angle
* `a`     - acceleration (throttle)

##### Model equations
<!---
=============================================================
Latex editor and graphics renderer
https://www.codecogs.com/latex/eqneditor.php

$\begin{aligned}
x_{t+1} &= x_t + v_t * cos(psi_t) * dt \\
y_{t+1} &= y_t + v_t * sin(psi_t) * dt \\
psi_{t+1} &= psi_t + v_t / Lf * delta_t * dt \\
v_{t+1} &= v_t + a_t * dt \\
cte_{t+1} &= (f(x_t) - y_t) + v_t * sin(epsi_t) * dt \\
epsi_{t+1} &= (psi_t - psi\_des_t) + v_t * delta_t / Lf * dt
\end{aligned}$

=============================================================
HTML simple render with subscripts
x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> * cos(psi<sub>t</sub>)
y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> * sin(psi<sub>t</sub>)
psi<sub>t+1</sub> = psi<sub>t</sub> + v<sub>t</sub> / Lf * delta<sub>t</sub> * dt
v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> * dt
cte<sub>t+1</sub> = (f(x<sub>t</sub>) - y<sub>t</sub>) + v<sub>t</sub> * sin(epsi<sub>t</sub>) * dt
epsi<sub>t+1</sub> = (psi<sub>t</sub> - psi_des<sub>t</sub>) + v<sub>t</sub> * delta<sub>t</sub> / Lf * dt
--->

<p align="center">
<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{120}&space;\large&space;\begin{aligned}&space;x_{t&plus;1}&space;&=&space;x_t&space;&plus;&space;v_t&space;*&space;cos(psi_t)&space;*&space;dt&space;\\&space;y_{t&plus;1}&space;&=&space;y_t&space;&plus;&space;v_t&space;*&space;sin(psi_t)&space;*&space;dt&space;\\&space;psi_{t&plus;1}&space;&=&space;psi_t&space;&plus;&space;v_t&space;/&space;Lf&space;*&space;delta_t&space;*&space;dt&space;\\&space;v_{t&plus;1}&space;&=&space;v_t&space;&plus;&space;a_t&space;*&space;dt&space;\\&space;cte_{t&plus;1}&space;&=&space;(f(x_t)&space;-&space;y_t)&space;&plus;&space;v_t&space;*&space;sin(epsi_t)&space;*&space;dt&space;\\&space;epsi_{t&plus;1}&space;&=&space;(psi_t&space;-&space;psi\_des_t)&space;&plus;&space;v_t&space;*&space;delta_t&space;/&space;Lf&space;*&space;dt&space;\end{aligned}" title="Model Equations" />
</p>

### Timestep Length and Elapsed Duration (N & dt)

Because the goal of Model Predictive Control is to optimize the control inputs [delta, a], an optimizer solves for a low cost vector of control inputs. The length of the optimized vector is determined by **N**, the number of timesteps. The tuned low-cost vector:

<!---
$$[delta_1, a_1, delta_2, a_2,\dots, delta_{N-1}, a_{N-1}]$$

[ delta<sub>1</sub>, a<sub>1</sub>, delta<sub>2</sub>, a<sub>2</sub>,..., delta<sub>N-1</sub>, a<sub>N-1</sub> ]
--->
<p align="center">
<img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{120}&space;\large&space;$$[delta_1,&space;a_1,&space;delta_2,&space;a_2,\dots,&space;delta_{N-1},&space;a_{N-1}]$$" title="Cost Vector of Control Inputs" />
</p>

Since **N** determines the number of variables optimized by the MPC, **N** directly influences the total computational cost. Additionally, the time horizon over which predictions are made is also determined by **N**, i.e., `N * dt`. The product of the two values of **N** and **dt** (the time elapsed between actuations) is the prediction time horizon. A reasonable horizon for prediction is a few seconds for a vehicle moving at high speeds because beyond the reasonable few seconds, the environment will have changed enough to render any further predictions into the future nonsensical. Thus, **N** in concert with **dt** are chosen to yield a prediction horizon of a few seconds as well as minimizing the computational costs, keeping the total number of actuator variables that must be tuned by the optimizer.

| N     | dt       | Notes: Refernce velocity was kept at 100 mph|
|------:|:---------|:--------------------------------------------|
| 10    |0.10      | Change in steering was too quick and erratic at some points along the track. Car went off-road. |
| 8     |0.10      | Smoother, but trajectory seemed more unstable; had difficulty matching the reference line. |
| 10    |0.15      | Larger dt means fewer actuations, thus lesser number of changes to steering and acceleration. Car successfully negotiated the entire track. |
| **10**| **0.14** | Larger dt improved accuracy, but slowed way down on sharp turns. Lowered dt a tiny bit to help with maintaining higher speed around the turns. Achieved up to 90 mph. |

### Polynomial Fitting and MPC Preprocessing

Waypoints provided by the simulator server are in the map coordinate system. The waypoints are first translated to the vehicle's coordinate system. Each of the vector of x's and vector of y's are put into respective `Eigen::VectorXd`s for consumption by the `polyfit()` function. A 3rd-degree polynomial is fitted to the translated waypoints. Using the fit polynomial, the `cte` is calculated by evaluating the polynomial using `polyeval()` function at x = 0, the current x position of the car. The `epsi` is also calculated:

```c++
double epsi = psi - atan(fitCurve_coeffs[1] + 2 * px * fitCurve_coeffs[2] + 3 * fitCurve_coeffs[3] * pow(px, 2));

// NOTE: the expression inside the atan function call is the derivative of the fit polynomial function
```
Assuming px = 0 and psi = 0, `epsi` reduces to the following:

```c++
double epsi = -atan(fitCurve_coeffs[1]);
```

At this point, the state (see Model section above) is then passed to the MPC to solve for the actuation values, `delta` and `a`. The MPC returns the solution prediction [`delta`, `a`, trajectory]:

<!---
$$[delta, a, trajx_1, trajy_1, trajx_2, trajy_2, \dots, trajx_N, trajy_N]$$
--->

<p align="center">
<img src="https://latex.codecogs.com/gif.latex?[delta,&space;a,&space;trajx_1,&space;trajy_1,&space;trajx_2,&space;trajy_2,&space;\dots,&space;trajx_N,&space;trajy_N]" title="Solution Prediction Vector" />
</p>

### Model Predictive Control with Latency

Latency is the time between when actuator commands are issued to the vehicle and the time the vehicle actuates its controls based on the issued commands. To simulate this delay in the controls being actuated, we manually put in a delay to sending control commands to the vehicle by putting the running thread to sleep for 100ms. Once the sleep cycle is complete, the control message is sent back to the simulator server for actuation.

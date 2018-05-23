

## Udacity SDCND - Term 2: MPC Project ##

### I. The Model

I have used **classroom model**.

i. State
 - x: position in x direction
 - y: position in y direction
 - psi: steering angle
 - v: velocity of the car
 - cte: cross-track error along the y axis
 - epsi: error in the steering angle
 
ii. Actuators
 - delta:  applied steering angle
 - a:  applied throttle
 
iii. Update Equations
 - x<sub>t</sub> = x<sub>t-1</sub> \* v<sub>t-1</sub> \* cos(psi<sub>t-1</sub>) \* dt
 -  y<sub>t</sub> = y<sub>t-1</sub> \* v<sub>t-1</sub> \* sin(psi<sub>t-1</sub>) \* dt
 -  psi<sub>t</sub> = psi<sub>t-1</sub> + (v<sub>t-1</sub>/Lf) \* delta<sub>t-1</sub> \* dt
 -  v<sub>t</sub> = v<sub>t-1</sub> + a<sub>t-1</sub> + dt
 -  cte<sub>t</sub> = (f<sub>t-1</sub> - y<sub>t-1</sub>) + (v<sub>t-1</sub> \* sin(epsi<sub>t-1</sub>) \* dt)
 -  epsi<sub>t</sub> = ((psi<sub>t-1</sub> - psides<sub>t-1</sub>) - ((v<sub>t-1</sub>/Lf) \* delta<sub>t-1</sub> \* dt))

**f** is the value of the 3rd degree polynomial representing the reference line at the current value of x.  
**psides** is the desired psi, which is the tangential angle of the derivative of the polynomial at that point.

### II. Timestep Length and Elapsed Duration

The final values chosen are **N=10** and **dt=0.1**.

If the value of N is too small, we cannot predict the future well. If value is too large then we may plan for a long future which not be what we are expecting. The values for N and dt are 10 and 0.1 respectively. These values were just a part of hit and trial process. I tested with 7/0.5; 9,0.25; 18,0.05 also in order to fix 10 and 0.1.

### III. Polynomial Fitting

The waypoint co-ordinates received from the simulator are first converted into cars co-ordinate system where car is the origin, I have done it in `Main.cpp::Lines 104 - 114`.  
The converted co-ordinates are fit to a polynomial at `Main.cpp::Lines 124` using the polyfit method.

### IV. Model Predictive Control with Latency

In order to account for the 100 ms latency, the initial state of the car supplied by the simulator is updated using the same model descibed above.   
Here, the **latency** period is used as the time gap **dt**.

Below is the code block from `Main.cpp`.

```cpp
const double current_px = 0.0 + v * act_latency;
const double current_py = 0.0;
const double current_psi = 0.0 + v * (-delta) / Lf * act_latency;
const double current_v = v + a * act_latency;
const double current_cte = cte + v * sin(epsi) * act_latency;
const double current_epsi = epsi + v * (-delta) / Lf * act_latency;
```


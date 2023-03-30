# Self-balancing-Two-wheeled-vehicle
Design the control system of a stationary self balancing two-wheeled vehicle

## Brief Introduction
We have the state space linear model for the two-wheeled vehicle: <br>
$\dot{x} = Ax + Bu$ <br>
$y = Cx$ <br>
The state variable is <br>
$x = ([car position, handle angle, bike angle, \dot{car position}, \dot{handle angle}, \dot{bike angle}])^T$

## Feedback Design (measure 6 state variavles)
Use the pole placement method and LQR to perform feedback control and observe transient respose and non-zero initial state with zero external inputs.<br>
Pole Placement: <br>
<img src="figures/pole_place_step_response.png" alt="pole_place_step_response" width="400" height="300" title="step response">
<img src="figures/pole_place_zero_input_response.png" alt="pole_place_zero_input_response" width="400" height="300" title="zero input response"> <br>
LQR: <br>
<img src="figures/LQR_step_response.png" alt="LQR_step_response" width="400" height="300" title="step response">
<img src="figures/LQR_zero_input_response.png" alt="LQR_zero_input_response" width="400" height="300" title="zero input response">

## Design State Observer (measure 3 variables)
Design a state observer, simulate the resultant observer-based LQR control system, monitor the state estimation error, investigate effects of observer poles on state estimation error and closed-loop control performance. <br>
<img src="figures/state_observe_error_signal.png" alt="state_observe_error_signal.png" width="1000" height="500" title="performance of state observe and error signal.png"> <br>

## Design the decoupling system
Interested in the outputs of car position and bike angle. Design a decoupling controller with closed-loop stability of the 2-input-2-output system.
<img src="figures/DP_step_response.png" alt="DP_step_response" width="400" height="300" title="step response">
<img src="figures/DP_zero_input_response.png" alt="DP_zero_input_response" width="400" height="300" title="zero input response">

## Servo control 
Assume the step disturbance for the two inputs, $𝑤 = ([−1, 1])^T$ takes effect from time $𝑡_𝑑 = 10𝑠$ afterwards. <br>
Build an observer to get the estimation of the six states of the system to feedback to the original system because of the noise of three sensors. <br>
The simulink module in MATLAB: <br>
<img src="figures/servo_control_system.png" alt="servo_control_system" width="800" height="600" title="servo_control_system"> <br>
The response of the observer-based servo controller <br>
<img src="figures/servo_control_response.png" alt="servo_control_response" width="800" height="400" title="servo_control_response">

## Set-point tracking analysis
Set different track points and analyze the response.<br>

## Finally
All the details of this project are shown in the pdf file including the theories and calculations.
The simulink model are uploaded in the simulink file.

If you think it is useful for you work, please give me a star.
If you find some problems in this project, you can leave the comment. Thank you !!

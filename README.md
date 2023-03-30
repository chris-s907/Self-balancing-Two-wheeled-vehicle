# Self-balancing-Two-wheeled-vehicle
Design the control system of a stationary self balancing two-wheeled vehicle

## Brief Introduction
We have the state space linear model for the two-wheeled vehicle: <br>
$\dot{x} = Ax + Bu$ <br>
$y = Cx$ <br>
The state variable is <br>
$x = ([car position, handle angle, bike angle, \dot{car position}, \dot{handle angle}, \dot{bike angle}])^T$

## Feedback Design (measure 6 state variavles)
Use the pole placement method and LQR to perform feedback control and observe transient respose and non-zero initial state with zero external inputs.

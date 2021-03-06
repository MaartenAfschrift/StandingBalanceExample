# Simulation standing balance

This repository contains matlab scripts and functions to:
- Derive the equations of motion of a double inverted pendulum model (Folder EoM)
- Optimize the gains of a full-state feedback controller to compensate for a support surface translation with minimal COM motion and joint moments.

## Description of the double inverted pendulum model

The model consists of two degrees of freedom representing the ankle ($q_1$) and hip ($q_2$) joint. The ankle angle is the angle between the leg and the vertical line, the hip angle is the relative angle between the leg and the trunk segment (0 corresponds to the leg and trunk being aligned).  Each degree of freedom is actuated by an ideal torque actuator (T). 

We derived the equations of motion in the script DoublePendulum_EOM_Largrange. We used this scrip to create the following functions:
- f_COMx : computes horizontal position of the whole body center of mass (CoM)
- f_COP: Computes the positio of the center of pressure (COP)
- f_Fy: computes the vertical ground reaction force (Fy)
- f_Tid: computes the inverse dynamic joint moments

These functions require (some) of the following input arguments
- q1 and q2: ankle and hip joint angle (in radians)
- qd1 and qd2: ankle and hip angular velocity (in radians)
- qdd1 and qdd2: ankle and hip angular acceleration (in radians)
- m: mass of the double pendulum (i.e mass subject) (in kg)
- l: length of the double pendulum (i.e. length subject) (in m)
- ap: acceleration of the motion base (in m/s2)

Note that the relative mass distribution and relative location of the COM in the segments is constant. Therfore, you only have to specify the total mass and total length. You can find more information on how to use these functions in the script Info.m


## Optimal feedback control

We use casadi to formulate the optimal control problem. You can download casadi here (https://web.casadi.org/get/) and simply install it by adding the casadi folder to your matlab path.

We used a similar approach as in [1] to predict postural responses to platform translations. In short, we optimize the feedback gains of a full-state feedback controller to minimize a weighted sum of (1) COM displacement (f_COMx.m) and (2) joint torques (f_Tid.m). 

$$ J(K) = w_1 COM^2 + w_2 T^2 + w_3 (K-K_{LQR})^2 $$

Note that we added a third term $(K-K_{LQR})^2$ to regularise the feedback gains. This term has a low weight ($w_3$) and keeps the feedback gains close to the LQR solution (i.e. stable feedback gains of the linearised system).

We used a direct collocation approach with an euler integration scheme to formulate the optimal feedback control problem, which was solved using ipopt.

## General description of the optimal control problem

minimize: $J(K,x,\dot{x})$ 

with respect to K, x, \dot{x} where x = [q1 q2 qd1 qd2] is the state vector

subject to:

- backward euler integration: $x_{i+1}-x_i = \dot{x}_i * dt$ 

- feedback control: $T = K x, with $T (x, \dot{x})$ the inverse dynamic joint moments (computed using f_Tid.m)

- COP in BOS: We added a constraint that the COP should remain within the functional base of support of the subject. This is needed to prevent the COP exceeding the dimensions of the foot (i.e. base of support), which is possible because we welded the feet to the ground in this simple model.

### Initial guess

We formulate the initial guess of the feedback gains and kinematics based on a foward simulation with LQR feedback gains. 

### Output
The results are extracted from the solution and saved in the matlab scruture R. At the end of the mainscript you can find some code to plot and visualise the optimal solution.

### Notes on problem formulation

Once you installed casadi, you should be able to run the script MainScript.m. You will see that we used opti-stack to formulate the optimal control problem. Opti is a compact syntax to define NLP's (non-linear programs). You can find more information here: https://web.casadi.org/blog/opti/. You'll see that this is very user friendly.


## References

[1] Afschrift et al. 2018, Increased sensory noise and not muscle weakness explains changes in non-stepping postural responses following stance perturbations in healthy elderly ([10.1016/j.gaitpost.2017.10.003](https://doi.org/10.1016/j.gaitpost.2017.10.003))

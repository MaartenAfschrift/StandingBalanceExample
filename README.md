# Simulation standing balance

This repository contains matlab scripts and function to:
- Derive the equations of motion of a double inverted pendulum model (Folder EoM)
- Optimize the gains of a full-state feedback controller to compensate for a support surface translation with minimal COM motion and joint moments.

## Description of the double inverted pendulum model

The model consists of two degrees of freedom representing the ankle ($q_1$) and hip ($q_2$) joint. The ankle angle is the angle between the leg and the vertical line, the hip angle is the relatie angle between the leg and the trunk segment.  Each degree of freedom is actuated by an ideal torque actuator (T). 

We derived the equations of motion in the script DoublePendulum_EOM_Largrange. We used this scrip to create the following functions:
- f_COMx : computes horizontal position of the whole body center of mass (CoM)
- f_COP: Computes the positio of the center of pressure (COP)
- f_Fy: computes the vertical ground reaction force (Fy)
- f_Tid: computes the inverse dynamic joint moments

These functions require (some) of the following in put arguments
- q1 and q2: ankle and hip joint angle (in radians)
- qd1 and qd2: ankle and hip angular velocity (in radians)
- qdd1 and qdd2: ankle and hip angular acceleration (in radians)
- m: mass of the double pendulum (i.e mass subject) (in kg)
- l: length of the double pendulum (i.e. length subject) (in m)
- ap: acceleration of the motion base (in m/s2)

Note The relative mass distribution and relative location of the COM in the segments is constant. Therfore, you only have to specify the total mass and total length. You can find more information on how to use these functions in the scrip Info.m


## Optimal feedback control

We use casadi to formulate the optimal control problem. You can download casadi here (https://web.casadi.org/get/) and simply install it by adding the casadi folder to your matlab path.

We used a similar approach as in [1] to predict postural responses to platform translations. In short, we optimize the feedback gains of a full-state feedback controller to minimize (1) deviations of COM position and (2) joint torques. 

$$ J(K) = w_1 COM^2 + w_2 T^2 + w_3 (K-K_{LQR})^2 $$

Note that we added a third term $(K-K_{LQR})^2$ to regularise the feedback gains. This term has a low weight ($w_3$) and keeps the feedback gains close to the LQR solution (i.e. stable feedback gains of the linearised system).

We used a direct collocation approach with an euler integration scheme to formulate the optimal feedback control problem, which was solved using ipopt.

## General description of the optimal control problem

optimize: $J(K)$ 

subject to:

- backward euler integration: $x(t+dt)-x(t) = \dot{x}*dt$

- feedback control: $T(t) = K x(t)$, with $T$ the inverse dynamic joint moment

- COP in BOS: We added a constraint that the COP should remain within the functional base of support of the subject. This is needed to prevent the COP exceeding the dimensions of the foot (i.e. base of support), which is possible because we welded the feet to the ground in this simple model.

### Initial guess

We formulate the initial guess of the feedback gains and kinematics based on a foward simulation with LQR feedback gains. 

### Output
The results are extracted from the solution and saved in the matlab scruture R. At the end of the mainscript you can find some code to plot and visualise the optimal solution.

### Notes on problem formulation

Once you installed casadi, you should be able to run the script MainScript.m. You will see that we used opti-stack to formulate the optimal control problem. Opti is an compact syntax to define NLP's (Non-linear programs). You can find more information here: https://web.casadi.org/blog/opti/. You'll see that this is very user friendly.

## Excersize

- Decrease the default functional base of support with 50% and with 80%. Discuss how the predicted motion changes and compare this to the results of [1].
- How does the height of the subject influence the optimal postural response ?



## References

[1] Afschrift et al. 2018, Increased sensory noise and not muscle weakness explains changes in non-stepping postural responses following stance perturbations in healthy elderly ([10.1016/j.gaitpost.2017.10.003](https://doi.org/10.1016/j.gaitpost.2017.10.003))
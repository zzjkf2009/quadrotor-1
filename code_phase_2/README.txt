This is project 1 phase 2. The model of the Quard is given. It required to fly through two
set of trajectories (circle and diamond). According to its kinematic and dynamic equations,
a controller is designed under the small angle assumptions for roll and pitch. 
More details about the dynamic and control can be find in: "Trajectory Generation and Control for Quadrotors, D. Mellinger"

Trajectory Generation:
The idea I uesd to design the trajectory in 3D is that the trajectory is a second order equation of time, the Quad accelerate 
at the first half of time and them decelerate at the second half. At the end, its velocity will decrease to zero, which means
it stops at the ending time in a state of hovering. 


Controller Generation:
As we learned from the leacture, the controller has two parts, one is its outer loop, position control. The other is the inner loop, 
attitude control. All of the equation is under the assumption that the roll and pitch angle is relatiave small (max 30 degree) and it 
always trying to maintain hovering. Based on that, we can derive the control equation and linearize them. 

Gains are tuned based on the simulation results, if the position or velocity is overshoot or oscilate too much, then decrease the kp, increse kd.
The velocity tracking for z axis in first (circle) trajectory needs more time to find the propriate gain value for the controller. 
The attitude gain for the controller can't be too samll in order to balance the drone
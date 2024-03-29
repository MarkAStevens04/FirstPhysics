# FirstPhysics
**= Purpose =**
The purpose of this project is to gain experience in modeling physical systems using computer science. 
This simulation is not meant to accurately predict real-world scenarios but is designed to provide results that mimic reality.
All methods are grounded in the physical laws of our Universe.

**= Overview of design =**
Because accuracy is not required, and simplicity is prioritized, the semi-implicit Euler method was chosen.
Each object in the simulation has attributes storing current position, velocity, and acceleration. 
Velocity is updated by the equation dv/dt = a. dt is the step size, a is acceleration, and dv is the change in velocity.
For every dt (or time step), dv is updated, and the object's velocity is modified. This velocity modification is used to update the position of the object.

For collisions between two balls, a brute-force method of checking overlap is performed. 
Every pair of balls is checked to see if the distance between the balls is less than the sum of their radii. 
If the distance is less than this, we update the velocities of the balls and teleport the balls directly out of each other to remove overlap.
We iterate through this process indefinitely.

**= Explanation of ball collision handling =**
The simulation assumes perfectly elastic collisions with no loss in energy. 
This allows for the equations for conservation of momentum and conservation of energy to be combined to deduce the new velocities.
Because the balls are not necessarily colliding on the axis of our reference frame (the x-y axis), we must 'tilt' our axes,
and then perform the velocity calculations in our new reference frame. After acquiring our new velocity, we 'un-tilt' our axes,
and update the velocities of the ball in the new reference frame.

**= Limitations =**
The Explicit Euler method is more accurate when dt, or the size of each time step, is decreased. 
Similar to Reiman sum-based estimations of integrals, a greater number of sub-steps results in more accurate models of reality.
Each step of the Explicit Euler method, however, is computationally expensive, as it performs many unnecessary calculations.
Thus, dt must remain large to perform at the necessary speed, which produces results with decreased accuracy.
If the change in position of a ball is great enough, it could phase through another ball, or even through the wall.

**= Future improvements =**
Ideally, the simulation would be modeled by solving a system of equations modeling the forces and momentum of each object.
Collision detection would only happen within certain 'chunks', thus avoiding needless computation.
The simulation would utilize GPU acceleration via CUDA to quickly perform simple calculations.
Collision detection would occur by tracing the path of the balls, thus ensuring no balls phased through each other.

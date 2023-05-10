# FirstPhysics
**= Purpose =**
The purpose of this project is to gain experience in modeling physical systems using computer science. 
This simulation is not meant to accurately predict real world scenarios, but is rather designed to provide results which look realistic.
All methods are, however, based in physical laws of the Universe.

**= Overview of design =**
Because accuracy is not required, and simplicity is prioritized, the explicit Euler method was chosen.
Each object in hte simulation has attributes storing current position, velocity, and acceleration. 
Velocity is updated by the equation dv/dt = a. dt is the step size, a is the acceleration, and dv is the change in velocity.
Every dt (or time step), the dv is updated, and the object's velocity is modified. This cascades down to the position of the object as well.

To handle 2 ball collisions, a brute-force method of checking overlap is performed. 
Every pair of balls is checked to see if the distance between the balls is less than the sum of their radii. 
If the distance is less than this, we update the velocities of the balls and teleport the balls directly out of each other to remove overlap.
We iterate through this process indefinitely.

**= Explanation of ball collision handling =**
The simulation assumes perfectly elastic collisions with no loss in energy. 
This allows for the equations for conservation of momentum and conservation of energy to be combined to find the new velocities.
Because the balls are not necessarily colliding on the axis of our reference frame (the xy axis), we must 'tilt' our axises,
then perform the velocity calculations in our new reference frame. After acquiring our new velocity, we 'un-tilt' our axises,
and update the velocities of the ball in the new reference frame.

**= Limitations =**
The Explicit Euler method is more accurate when dt, or the size of each time step, is decreased. 
Similar to Reiman sum based estimations of integrals, a greater number of sub-steps results in more accurate models of reality.
Each step of the Explicit Euler method, however, is computationally expensive, as it performs many unnecessary calculations.
Thus, dt must remain large to perform at the necessary speed, but we get un-realistic results.
If the change in position of a ball is great enough, it could phase through another ball, or even through the wall.

**= Future improvements =**
Ideally, the simulation would be modeled by solving a system of equations modelling the forces and momentum of each object.
Collision detection would only happen within certain 'chunks', thus avoiding needless computation
The simulation would utilize the GPU to quickly perform simple calculations
Collision detection would occur by tracing the path of the balls, thus ensuring no balls phased through each other

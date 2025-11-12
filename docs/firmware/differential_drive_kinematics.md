# Differential Drive Kinematics

## Goal

Convert G-code G1 commands, which specify relative changes in tool head position in the $X$ and $Y$ directions (mm), into wheel angular velocities for a differential drive robot.

## Method

This method uses linear interpolation between points. This may be extended to cubic hermite interpolation in the future.

## Derivation

$$
(\Delta x, \Delta y) \;\rightarrow\; (d, \Delta \theta) \;\rightarrow\; (v_l, v_r) \;\rightarrow\; (\omega_l, \omega_r)
$$


### 1. Cartesian Δ to Polar Δ

Each change in position can be decomposed into two movements:
1. A change in heading angle, $\Delta \theta$
2. A change in path length, $d$

$$
d = \sqrt{(\Delta x)^2 + (\Delta y)^2} \\
\theta' = \operatorname{atan2}(\Delta y, \Delta x) \\
\Delta \theta = \theta' - \theta
$$


### 2. Polar Δ to Differential Wheel Displacements

Let $r_{\text{doodlebot}}$ represent half the wheelbase, i.e., the distance from the robot’s center to each wheel.  

Then the displacement of each wheel is:

$$
d_l = d - \Delta \theta \, r_{\text{doodlebot}} \\
d_r = d + \Delta \theta \, r_{\text{doodlebot}}
$$


### 3. Wheel Displacements to Linear Velocities

The stepper control period $T$ determines how long the motion should take. Let the control frequency be $f = \frac{1}{T}$.

$$
v_l = d_l \, f \\
v_r = d_r \, f
$$

This is where rate limiting must occurr. The physical speed limit of the DoodleBot is surely lower than the stepper motor or driver’s angular limit, as specified by the datasheet. If either $v_l$ or $v_r$ exceeds the maximum allowable linear velocity, they are broken into multiple movements of mangeable velocity to achieve the commanded distance. Subsequent transformations are applied for each new movement.


### 4. Linear to Angular Velocity

The standard kinematic relationship is:

$$
v = r_{\text{wheel}} \, \omega
$$

Rearranging for $\omega$ gives:

$$
\omega = \frac{v}{r_{\text{wheel}}}
$$

Thus, for each wheel:

$$
\omega_l = \frac{v_l}{r_{\text{wheel}}}, \quad
\omega_r = \frac{v_r}{r_{\text{wheel}}}
$$


### 5. Radians to Degrees (Stepper Driver Input)

Most stepper drivers accept angular velocities in degrees per second, so convert as:

$$
\omega_{l,°} = \omega_l \cdot \frac{180}{\pi}, \quad
\omega_{r,°} = \omega_r \cdot \frac{180}{\pi}
$$


## Implementation

The kinematics are implemented in `navigation.cpp` in the `MotionPlanner` class.

Step 1 happens in `interpolate()` which converts G-code coordinates to distance and angle commands. This is separated in the case of another interpolation method besides linear being used, this will be the only function that needs to change.

Steps 2-5 happen in `discretize()` which converts those commands to stepper motor velocities.

Physical constants like wheel radius and robot dimensions are defined in `navigation.h`.

Rate limiting occurs at Step 3 by splitting high velocities into multiple smaller movements.
# r3

`lib.controller.r3` contains control methods for three dimensional
coordinates, where the three coordinates are orthogonal.  Our high-level
swerve control uses this sort of method, even though the SE(2) group
of Pose2d is not R3.  It is *tangent* to R3, and if the control
errors are small, R3 is an acceptable approximation to SE(2).

The same controllers can be used for any problem with three independent
dimensions, e.g. a planar mechanism like the 2025 elevator/arm.

There's currently only one feedback controller implementation,
which uses velocity feedforward, and proportional feedback on position and velocity.
In the past, we had many more kinds of controllers, for example, pursuit followers,
LQR, min-time MPC, whatever.  Some of these ideas can be found in previous-year's
studies.  We found that complexity in control didn't really help anything,
and it was hard to understand, debug, and tune.

# geometry

the `lib.geometry` package includes types and utilities useful for
spatial reasoning.

Some important highlights:

For the geometry of 2d poses in the global reference frame, we use the
WPI Pose2d for position, and we have our own classes for velocity and acceleration.

Pose2d lives in the SE(2) Lie group, which represents rigid-body motion in 2
dimensions, i.e. x, y, and theta, in a smooth differentiable manifold. There
is an obvious coupling between the rotation and cartesian components here,
and the correct derivative takes this into account: see Pose2d.exp() and
Pose2d.log().
 
We do not do that. We treat global velocity and acceleration as if all the
components were independent, i.e. using the R3 vector space,
not the SE(2) Lie group.

Why do we do that?  Because at large scales, we think in R3, e.g. we define
trajectories and means to follow them without worrying about the coupling
in SE(2).  We *do* handle SE(2) correct at the smallest scale, one
robot-clock step at a time, where, for example, drivetrain actuation
needs to be correctly "discretized" so that the constant-twist paths
the robot follows (approximately) during a single time step end up
where they're supposed to be.

There is a good discussion of the math, applied to robotics, here:
 
 * https://gtsam.org/2021/02/23/uncertainties-part1.html
 * https://docs.resim.ai/open-core/transforms/liegroup_derivatives/
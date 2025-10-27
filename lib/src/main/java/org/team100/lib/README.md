# lib

The `lib` package contains library code we can use every year.

Everything in here needs to be reviewed and covered with tests.

If you're trying to learn about the library, you should start with the README in each package.

__Packages of note__

* [coherence](coherence/README.md)
  * A discrete clock and cache intended to manage observations (so they all represent the same instant) and anything else that depends on time (e.g. steps along a profile).
* [commands](commands/README.md)
  * High level navigation commands, using manual control, profiles, and trajectories.
* [config](config/README.md)
  * The most interesting config is `Identity`, which is an enum of RoboRIO
  serial numbers.  We use this identity to differentiate robot hardware within
  a single code-base.
* [controller](controller/README.md)
  * Feedback controllers for one dimension and three independent dimensions.  We use the latter for 2d-with-heading poses.  Some controllers do feedforward, some do feedback, some do both.
* [encoder](encoder/README.md)
  * Support for various ways to measure the position of a joint, including via combination of multiple sensors, e.g. an integrated incremental motor encoder and an external absolute position sensor.
* [examples](examples/README.md) 
  * A few illustrations of using library features.
* [experiments](experiments/README.md) 
  * switches that control optional features using dashboard selectors.
* [hid](hid/README.md) 
  * We push the "control domain" (what each button *means*) into the controller API, and allow hot-swapping controllers.
* [localization](localization/README.md)
  * We use camera sightings of canonical AprilTags, combined with odometry and gyro input, for full-field localization, used in navigation and pose-aware commands, e.g. maintaining a shooter aim point.
* [logging](logging/README.md) 
  * Loggers are passed through constructors, so the tree in Network Tables mirrors the construction graph, as a starting point.
* [motion](motion/README.md) 
  * High-level classes for motion control and kinematics, including support for both onboard computation (computed in Java) and offboard computation (partially computed by "smart" motor controllers).  There are packages for "mechanisms" (which include gear ratios), and "servos" (which include feedback and feedforward control).
* [motor](motor/README.md) 
  * Wrappers for all the motors we use (including simulated motors), so they can be handled uniformly by the motion classes above.
* [optimization](optimization/README.md)
  * solvers used in applications like inverse kinematics, notably an implementation of Newton's method
for finding the zero of a function.
* [profile](profile/README.md)
  * Constrained motion in 1d and 2d-with-heading.  These should be used for all motion where a trajectory would be too expensive to compute on the fly.
* [reference](reference/README.md)
  * Team 100 reference generators.  These use any sort of reference source, e.g. profile or trajectory, and produce "current" and "next" setpoints.  The reason for these classes is to make sure the time-alignment of observation and control are done correctly.  They use the `coherence` machinery mentioned above.
* [targeting](targeting/README.md)
  * A fundamental robot navigation task is to identify targets to drive to.  The `lib.targeting` package turns camera observations into field-relative targets.
* [trajectory](trajectory/README.md)
  * A trajectory is a path based on splines, with a precalculated schedule meeting timing constraints.  Trajectories are good for paths that require curves around known obstacles.  Simple trajectories are not *that* time-consuming to create, so can be used on-the-fly in some cases.

Other notable topics about the library:

__Simulation__

Our approach to simulation is completely different from the WPILib approach,
in two important ways. First, WPILib attempts to simulate realistic mechanism
physics: we do not.  Second, WPILib attempts to involve vendors (e.g. REV) in
the simulation of their hardware: we do not.

Our approach to simulation is to use simulated motors that respond instantly
to their inputs.  The expectation is that inputs will be _feasible_.  Team
100 simulation is not useful to understand mechanism physics, it is only
useful for higher-level concerns, like designing paths to follow.

Our simulated motor implementations have nothing to do with the vendor
implementations; we use `Identity` in the constructor tree to select the
simulated implementation (for the `BLANK` identity).

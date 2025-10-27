# trajectory

The `lib.trajectory` package represents an path in SE(2) and a schedule.

`Trajectory100` is adapted from 254.  Trajectories are holonomic,
i.e. handle course and heading separately, so they have four dimensions (x, y, heading, course), and "point" below means four-dimensional holonomic state.

A good entry point to experiment with trajectories is `TrajectoryPlanner`.  Try `restToRest()` between two poses.

The process of constructing a trajectory has three stages:

1. Construct a list of splines.  Each spline joins two points smoothly.  The spline parameter has no physical meaning, it's just 0.0 on one end and 1.0 on the other.  See the `lib.trajectory.path.spline` package.

2. Construct a list of points along the spline, such that straight lines connecting the points ("secant lines") don't deviate too much from the true spline. (This uses recursive bisection.)  These points will be close together where the curvature is high, and far apart along straighter sections of the spline.  The list is created by `PathFactory`, producing `Path100`, which integrates along the list to find the distance.  See the `lib.trajectory.path` package.

3. Construct a list of points interpolated along the secant lines, such that the points aren't too far apart.

4. Using a list of kinodynamic constraints (see `lib.trajectory.timing`), assign a time for each point.  The resulting list of `TimedPose`s is created by `ScheduleGenerator`, producing `Trajectory100`.

To use a trajectory, you `sample()` it, with time (in seconds) as the parameter.  The resulting `TimedPose` is interpolated between from the list above.

If you want to use these trajectories for non-holonomic (e.g. "tank") drivetrains, it will work well enough to set the course and heading to be the same at each waypoint.
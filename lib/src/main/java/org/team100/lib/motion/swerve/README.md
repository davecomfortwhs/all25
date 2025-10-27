# swerve

The swerve package has several notable parts:

* `kinodynamics` describes the kinematics of the drivetrain (i.e. what to do with
each wheel in order to go a specific direction), and also a few aspects of dynamics,
including velocity and acceleration limits.  In the `kinodynamics.limiter` package,
there are methods to constrain control input to the feasible region.
These limits may be due to motor behavior, or to avoid tipping over, for example.
* `module` describes the module mechanisms, and has a few containers useful for
controlling them.

The subsystem has two layers:

* `SwerveLocal` is only aware of robot-relative control, and performs inverse kinematics
to execute it in SE(2), using discretization.
* `SwerveDriveSubsystem` is aware of field-relative continuous control in R3.

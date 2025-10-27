# Manual Driving

The `DriveManually` command uses a selectable control method, using one of these interfaces:

* `ModuleStateDriver` maps control input directly to module motor output.
This is only useful for testing, and there's currently only one implementation.

* `ChassisSpeedDriver` maps control input to *robot-relative* motion.
This is only useful for testing, and there's currently only one implementation.

* `FieldRelativeDriver` maps control input to *field-relative* motion.
This is what we actually use.  The simplest implementation is `ManualFieldRelativeSpeeds`
which is pure manual, but there are a few implementations that supplement manual control
with some sort of automation:

  * `ManualWithProfiledHeading` accepts both manual and "snap" input for heading, using
  `HeadingLatch` to select which input to use.  Uses a profile and feedback to control
  rotation when in "snap" mode.  The rotational profile is slow when the robot is driving fast.

  * `ManualWithFullStateHeading` uses a very simple control law:
  u = K<sub>1</sub> &theta; + K<sub>2</sub> &omega;. There's no profile timing to
  worry about.  The current implementation uses fixed gains, but might be improved
  by gain scheduling according to robot velocity, as above.
  
  * `ManualWithTargetLock` controls the robot rotation so that it maintains zero relative
  bearing towards a target.  This is for shooting.
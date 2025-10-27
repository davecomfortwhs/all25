# controller

The `lib.controller` package contains code that controls mechanisms, i.e. attempts
to drive a mechanism to a target position and/or velocity, by measuring the
actual mechanism state and applying some control law.

There are two packages here, `r1` for single-dimension control, and `r3` that
bundles together three independent dimensions, for drivetrain or planar
mechanism control.
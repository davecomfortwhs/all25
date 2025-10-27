# coherence

The `lib.coherence` package implements Team 100's approach to discretizing
time and making observations consistent within time steps.

Without special attention:

* The WPILib clock is essentially continuous, which means that things like
  control time steps will depend on Java timing, which introduces jitter
  into whatever you're using the time-step for, e.g. differentiation.
* The instant in time represented by each observation also depends on
  Java timing; the very same quantity can have two different values
  within the same time-step depending on when it is sampled.

To avoid these issues:

* We use a discrete clock called `Takt`.  The name comes from factory
  synchronization, it's a sort of "heartbeat."  The Takt time is
  sampled at the start of each main loop. The goal is for Takt to represent,
  as nearly as possible, the instant of time when the hardware interrupt fires.
* Observations are registered with `Cache`, and refreshed at the
  start of every main loop.  The goal is for the measurements to represent,
  as nearly as possible, the state of the world at the time of the hardware
  interrupt.

Cache updates run in two phases: first all the caches are invalidated.  Then all
the caches are refreshed, one at a time.  Some caches might depend on others.
These dependencies are handled through the normal flow of user code.
Refreshing one cache may also refreshe a dependency as a side-effect, so that
when the central cache refresher gets to the second one, it's already done.
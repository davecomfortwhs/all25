# Getting started with the library

The library is intended to support student work at a high level
of abstraction, as a supplement to WPILib.

To get started, it would be good to look through the code from 2025,
starting with `Robot`, but there's a lot there.

Here are some of the highlights for a simple mechanism:

## Create a subsystem

A simple example would be the `Climber` from 2025.
It comprises single degree of freedom, implemented using the Team100
"Servo" concept.  A servo combines a motor, some gears, a sensor,
a motion profile, and feedback control into a single interface, so
your code just needs to call methods like `setPositionProfiled()`.

There are two categories of things to worry about in the subsystem:

* __Match the physical robot__. Set up the correct motor CAN ids,
sensor offsets, gear ratio, and motion limits.
* __Tune current limits, profiles, and feedback__.  It's good to start with
very low current limits, very slow profiles, and very gentle feedback,
when you're first "turning up" a new subsystem.  You'll gradually
raise all these numbers as you get comfortable with the system.

The 2025 Climber also illustrates some common patterns in Team100 code.

* __Identity-dependent instantiation for simulation.__  In the constructor,
we use the RoboRIO Identity (based on its serial number) to decide what
specific setup to use.  It is often the case that we have multiple
similar-but-not-identical swerve drivetrains, for example.  In the
Climber case, we just use the Identity to know if we're running in
simulation or not.  If so, we use simulated motors and sensors.

* __Command factories.__  The subsystem provides a few methods, each
of which returns a `Command` that makes the subsystem do something,
for example `goToIntakePosition()`.  You assemble these commands into
the behaviors you want.

## Instantiate your subsystem in Robot.java

In the `Robot` constructor, you create an instance of your subsystem.
Sometimes we use the identity-dependent pattern here too, but not always.

Usually major mechanisms are `Robot` fields, but not always.

```java
private final Climber m_climber;

public Robot() {
    m_climber = new Climber(logger, new CanId(13));
```

## Set "stop" as the default command

Usually a mechanism default command is something like "stop".
Almost always, "stop" should mean "turn off the motors" rather
than "hold the current position," because it's safer.

```java
    m_climber.setDefaultCommand(
        m_climber.stop()
            .withName("climber default"));
```

## Bind some buttons

Our usual button binding pattern uses the `whileTrue` semantics
from WPILib: a command runs as long as a button is held, and if you
let go, the command is cancelled.  There are other semantics we
could use, for example you could use one button to start the command,
and another one to stop it, or you could use the same button for both
purposes as a kind of toggle.  The reason we use `whileTrue` is
for safety: if something bad happens, just let go and everything
should go back to the `stop()` default command.

So for the climber, a simple binding would be:

```java
    whileTrue(operator::x,
            m_climber.goToIntakePosition());
```

It's common to want button semantics like "do a thing while I
hold the button, and then do another thing when I let go."  For
that, we use `whileTrue` chained with `onFalse`:

```java
    whileTrue(operator::x,
            m_subsystem.grabAndHold()
    ).onFalse(
            m_subsystem.eject().withTimeout(0.5));
```

For these "when I let go" commands, be sure to include an explicit
timeout, for safety.

## Bonus: Composite commands

It is common to want multiple things to happen at once, and for
that, we use WPILib command compositions.  We put these in separate
"factory" classes, sometimes just a collection of static methods,
in order to keep `Robot` from getting too big.  A good example is
the first stage of climbing:

```java
parallel(
        climber.goToIntakePosition(),
        intake.intake(),
        mech.climbWithProfile()
    ).withName("climb intake")
```
This is a simple parallel composition that does three things
at once: stick out the climber, spin the intake rollers,
and move the big arm ("mech") out of the way.  Note the use of
`withName` so that the command scheduler widget in glass shows
something useful.

## Bonus: MoveAndHold

The WPILib compositions have a quirk that's important to understand:
when you run multiple commands in parallel, they can finish at
different times.  The composition continues to run until the slowest
command finishes.  For example, say you have two subsystems and
two commands in parallel:

```java
parallel(
        drive.toStation(),
        arm.out()
)
```

Imagine that the `drive` part takes a few seconds, but the `arm` part
completes right away.  What happens to the `arm` after that?  The
answer is that the arm gets no input at all. What
that usually means is that mechanisms will droop under the influence
of gravity.

To address this issue, our convention is for all our commands to
never finish: `Command.isFinished()` is always the default, which
returns false.

Instead, we extend Command with `MoveAndHold` which defines
`isDone()`, and we use that to set end conditions for the components
of parallel composites.  `MoveAndHold` also defines `toGo()` which
allows starting parallel commands before the first one is done.

For example, a scoring routine that involves driving, raising the
end effector when close to the goal, then actuating it when the both
driving and raising are done, might be written like this:

```java
var drive = drivetrain.driveToSomeGoal();
var up = elevator.goUp();
var actuate = endEffector.doTheThing();

parallel(
    drive,
    waitUntil(() -> drive.toGo() < 1).andThen(up),
    waitUntil(() -> (drive.isDone() && up.isDone())).andThen(actuate)
).until(() -> (drive.isDone() && up.isDone() && actuate.isDone()))
```

Something similar is done in places like `ScoreL1Smart` and similar
classes from 2025.

Using these custom end conditions and WPILib composites, you can
create quite complex behaviors.

## Bonus: Driving around

The swerve drive has quite a few components; you'll have plenty
of time to explore them all, but probably the first thing you'll
want to do is make the robot drive around, in particular drive
autonomously.

A good place to start for that would be `DriveToPoseWithProfile`,
which is a command that takes a `Supplier` of `Pose2d` goals, so you
could use it like this in `Robot`, as a one-line binding:

```java
    whileTrue(operator::x,
        new DriveToPoseWithProfile(
            logger, drive, controller, profile, 
            ()->new Pose2d(1, 2, new Rotation2d()));
);
```

This will drive to whatever pose you supply.

## Bonus: Visualizations

In `Robot` you'll see some visualizations created, e.g. for the climber
and the big elevator/arm mechanism.  These use the `Mechanism2d` widget
from WPILib that works in glass and simulation, so you can see how
your mechanisms are working with your commands.
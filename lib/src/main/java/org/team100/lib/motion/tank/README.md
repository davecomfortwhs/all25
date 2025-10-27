# tank

Tank drive examples, used in demo and rookie robots.

There are two implementations.

The first one, `BareMotorTank`, uses brushed motors without encoders,
for simplicity and economy.  This type of motor can't easily be used
with closed-loop velocity control.

The second one, `ServoTank`, uses brushless sensored motors which
can be controlled in closed-loop, so we use the Team 100 motion
control stack.
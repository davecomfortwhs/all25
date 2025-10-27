# Conventions

Team 100 code uses a number of conventions in style and method.  These conventions are important,
because they make the code easy to read.  Code with a lot of variety in style is not easy to
read.

## Style

Our naming conventions are similar to WPILib:

* Local variables are camel case: ```double myNumericThing;```
* Members are camel case starting with "m_", so, ```double m_thingToo;```
* Constants are upper case, so ```private static final double I_AM_CONSTANT = 1.0;```
* Use the minimum appropriate visibility.  for most members, this is ```private.```
* Use ```final``` liberally.  most members should be ```final```.  Favor immutability in general: it saves you from mysteries at runtime.
* Package names are lower case, class names are title case.

The most important aspect of Team 100 code is simplicity.  Classes should be small.  Methods should be short.

Use names that describe the meaning of what you're doing.  If you change the meaning, change the name.  If the meaning is really obvious, it's ok to use really short names like ```x```.

Avoid duplicative class names.  If you really want to use a name that is also used elsewhere, append "100" to it, e.g. there's a "MecanumDrive" in WPILib, so ours is called "MecanumDrive100".

When you have a lot of members and constructor arguments, put the lists in the same order.

When you have things that have a natural order, like "x" and "y", or "from" and "to", use that order.

Put variable initialization as close to use as possible.  The worst thing is to initialize all the variables at the top of a method, and then sprinkle the usage throughout the body.  Don't do that.

Don't repeat computation. If you use the same expression twice in a method, assign it to a local variable.

If you have a lot of initialization to do in one method, it's ok to use anonymous blocks to limit scope.

We indent four spaces, never tabs.  Use the auto-formatter.

## Comments

Use comments to describe why you're doing something, but don't use comments to simply explain what the code
is doing.  The code should be really obvious by reading it.  Using appropriate names really helps.  Using short
methods helps.

It's a good practice to comment the "public" part of your classes, so that VSCode pops up useful tooltips
about it. Use the "javadoc" style comment (/** ... */) above the class definition and each public method.

Don't combine comments and code on the same line, unless the line is very short.  VSCode's formatter will totally
mess up your comment otherwise.  Put your comments *above* the line they describe.

## Infrastructure

A common pattern is to print stuff to the console, if some sort of "debug" mode is engaged.  There are currently two patterns for that, a static boolean, or an interface that includes the printing methods -- the latter is preferred.

## Branching

If you know how to do it, it's encouraged to work on a branch in your fork, so that you can switch branches to work on more than one thing at a time.  If you don't know how to do it, don't worry about it.

## Units

We only ever use SI units: meters, radians, seconds, Newtons, and combinations thereof.  Never use inches or feet or degrees.

## Command composition

Keep commands simple and perpetual.  Expose "end" conditions that you can compose.  Don't embed timer-like features in your commands, just use WaitCommand.

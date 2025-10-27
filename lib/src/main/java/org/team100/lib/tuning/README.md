# tuning

In the past, Team 100 has made all adjustable constants *really constant*
in the code.  The benefit is that there's just one way to adjust something.
The cost is that every change requires a deploy cycle, so tuning can be
slow.

This package contains `Mutable` which allows adjustment on the fly, using
the normal Glass Network Tables interface.

Values do not survive restarts, so be sure to take notes, and add the values
you like into the code as the default value.
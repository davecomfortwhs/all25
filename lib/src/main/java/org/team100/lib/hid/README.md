# hid

The `lib.hid` package is for human interface devices like joysticks, button boards, etc.

The control classes used to be "semantic" controls, i.e. they'd have methods like
`climb()`, but the extra layer there didn't seem that helpful; comments would appear
at the call site (in `RobotContainer`) copying the binding.  So now they just rename
the methods to be a little nicer to read, and they include a little bit of logic for
multi-input grouping and thresholding.

The control classes also used to support hot-swapping, but we never used that, and the
extra layer of indirection was an obstacle, so I took it out.

## Custom controls

See `all25/console` for some Arduino HID projects for custom controls,
e.g. the MIDI controller, or the many-many button controller used in 2025.
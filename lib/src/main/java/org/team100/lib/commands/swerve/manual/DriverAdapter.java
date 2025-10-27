package org.team100.lib.commands.swerve.manual;

import org.team100.lib.hid.Velocity;
import org.team100.lib.state.ModelR3;

/**
 * Adapts various types of manual drivers so `DriveManually` can switch between
 * them easily.
 */
public interface DriverAdapter {
    void apply(ModelR3 s, Velocity t);

    void reset(ModelR3 s);
}
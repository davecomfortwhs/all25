package org.team100.lib.reference.r3;

import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;

/**
 * A source of references for control in R3, useful for drivetrain or
 * planar mechanism.
 */
public interface ReferenceR3 {

    /**
     * Make current() return the starting point, which might be informed by the
     * supplied measurement, or not.
     */
    void initialize(ModelR3 measurement);

    /**
     * Reference for the current time. This can be null (e.g. between instantiation
     * and initialization).
     */
    ModelR3 current();

    /**
     * Reference for 0.02 sec in the future. This can be null (e.g. between
     * instantiation and initialization).
     */
    ControlR3 next();

    /**
     * The reference series has reached the end, and will keep returning the same
     * (goal) reference.
     */
    boolean done();

    /**
     * For reference sources that have endpoints, what is the endpoint?
     */
    ModelR3 goal();
}

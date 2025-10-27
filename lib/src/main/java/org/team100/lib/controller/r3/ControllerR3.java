package org.team100.lib.controller.r3;

import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;

/**
 * Feedback and feedforward control.
 */
public interface ControllerR3  {

    /**
     * Feedback should compare the current-instant measurement to the
     * current-instant reference.
     * 
     * Feedforward should be looking at the next-step reference.
     * 
     * Previous versions of this method used a single reference for both.
     * 
     * @param measurement      Current measurement state in field coordinates
     * @param currentReference Current reference state i.e. setpoint
     * @param nextReference    Reference for dt in the future, used for feedforward.
     * @return Control output, should be given to
     *         SwerveDriveSubsystem.driveInFieldCoords() or something similar.
     */
    GlobalVelocityR3 calculate(
            ModelR3 measurement,
            ModelR3 currentReference,
            ControlR3 nextReference);

    /**
     * True if the error is within tolerance of the reference. The definitions of
     * "error" and "tolerance" here are implementation-specific.
     */
    boolean atReference();

    /**
     * Reset controller state, e.g. velocity error.
     */
    void reset();

}

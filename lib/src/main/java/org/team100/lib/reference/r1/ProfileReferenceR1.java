package org.team100.lib.reference.r1;

import org.team100.lib.state.Model100;

/**
 * Provides current and next references for servos.
 * 
 * Goal must be set prior to initialization.
 * 
 * NOTE: this class doesn't know anything about angle wrapping.
 */
public interface ProfileReferenceR1 {

    void setGoal(Model100 goal);

    void init(Model100 measurement);

    SetpointsR1 get();

    /** The profile has reached the goal. */
    boolean profileDone();
}

package org.team100.lib.commands.swerve.manual;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Remembers the most recent desired heading, substituting null if there's any
 * dtheta input.
 */
public class HeadingLatch {
    private static final double unlatch = 0.01;

    private Rotation2d m_desiredRotation = null;

    /**
     * @param maxARad_S2 supply an acceleration that matches whatever profile or
     *                   expectations you have for angular acceleration.
     */
    public Rotation2d latchedRotation(
            double maxARad_S2,
            Model100 state,
            Rotation2d pov,
            double inputOmega) {
        if (Math.abs(inputOmega) > unlatch) {
            // if the driver is trying to drive, then let them
            m_desiredRotation = null;
        } else if (pov != null) {
            // if the driver is trying to snap, then let them
            m_desiredRotation = pov;
        } else if (m_desiredRotation == null &&
                Experiments.instance.enabled(Experiment.StickyHeading)) {
            // if the driver is providing no input, and there's no pov,
            // then use the current heading as the sticky heading.
            // give the robot a chance to slow down to avoid overshoot.
            double t = Math.abs(state.v()) / maxARad_S2;
            m_desiredRotation = new Rotation2d(state.x() + state.v() * t / 2);
        }
        return m_desiredRotation;
    }

    public void unlatch() {
        m_desiredRotation = null;
    }
}

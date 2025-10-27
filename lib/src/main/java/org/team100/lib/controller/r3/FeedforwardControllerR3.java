package org.team100.lib.controller.r3;

import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;

/**
 * A controller that doesn't do anything except return the "next" setpoint. This
 * is appropriate if feedback control is outboard.
 */
public class FeedforwardControllerR3 implements ControllerR3 {
    private final double m_xTolerance;
    private final double m_thetaTolerance;
    private final double m_xDotTolerance;

    private ModelR3 m_measurement;
    private ModelR3 m_currentReference;

    public FeedforwardControllerR3(
            double xTolerance,
            double thetaTolerance,
            double xDotTolerance) {
        m_xTolerance = xTolerance;
        m_thetaTolerance = thetaTolerance;
        m_xDotTolerance = xDotTolerance;

    }

    @Override
    public GlobalVelocityR3 calculate(ModelR3 measurement, ModelR3 currentReference, ControlR3 nextReference) {
        m_measurement = measurement;
        m_currentReference = currentReference;
        return nextReference.velocity();
    }

    @Override
    public boolean atReference() {
        if (m_currentReference == null) return false;
        if (m_measurement == null) return false;
        ModelR3 error = m_currentReference.minus(m_measurement);
        return error.translation().getNorm() < m_xTolerance &&
                Math.abs(error.rotation().getRadians()) < m_thetaTolerance &&
                error.velocity().norm() < m_xDotTolerance;
    }

    @Override
    public void reset() {
        //
    }

}

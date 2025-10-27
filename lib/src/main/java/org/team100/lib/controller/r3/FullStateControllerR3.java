package org.team100.lib.controller.r3;

import org.team100.lib.geometry.GlobalDeltaR3;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.GlobaDeltaR3Logger;
import org.team100.lib.logging.LoggerFactory.GlobalVelocityR3Logger;
import org.team100.lib.logging.LoggerFactory.ControlR3Logger;
import org.team100.lib.logging.LoggerFactory.ModelR3Logger;
import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;

/**
 * Velocity feedforward, proportional feedback on position and velocity.
 */
public class FullStateControllerR3 implements ControllerR3 {
    private static final boolean DEBUG = false;
    private final ModelR3Logger m_log_measurement;
    private final BooleanLogger m_log_atPositionReference;
    // private final BooleanLogger m_log_atVelocityReference;

    private final DoubleLogger m_log_PositionError;
    // private final DoubleLogger m_log_VelocityError;

    private final ModelR3Logger m_log_currentReference;
    private final ControlR3Logger m_log_nextReference;
    private final GlobalVelocityR3Logger m_log_u_FF;
    private final GlobaDeltaR3Logger m_log_position_error;
    private final GlobalVelocityR3Logger m_log_u_FB;
    private final GlobalVelocityR3Logger m_log_velocity_error;
    private final GlobalVelocityR3Logger m_log_u_VFB;

    private final double m_kPCart;
    private final double m_kPTheta;
    private final double m_kPCartV;
    private final double m_kPThetaV;
    private final double m_xTolerance;
    private final double m_thetaTolerance;
    private final double m_xDotTolerance;
    // private final double m_omegaTolerance;

    private boolean m_atReference;

    public FullStateControllerR3(
            LoggerFactory parent,
            double kPCart,
            double kPTheta,
            double kPCartV,
            double kPThetaV,
            double xTolerance,
            double thetaTolerance,
            double xDotTolerance,
            double omegaTolerance) {
        LoggerFactory log = parent.type(this);
        m_log_measurement = log.modelR3Logger(Level.DEBUG, "measurement");
        m_log_currentReference = log.modelR3Logger(Level.DEBUG, "currentReference");
        m_log_nextReference = log.controlR3Logger(Level.DEBUG, "nextReference");
        m_log_u_FF = log.globalVelocityR3Logger(Level.TRACE, "u_FF");
        m_log_position_error = log.globalDeltaR3Logger(Level.TRACE, "positionError");
        m_log_u_FB = log.globalVelocityR3Logger(Level.TRACE, "u_FB");
        m_log_velocity_error = log.globalVelocityR3Logger(Level.TRACE, "velocityError");
        m_log_u_VFB = log.globalVelocityR3Logger(Level.TRACE, "u_VFB");

        m_log_atPositionReference = log.booleanLogger(Level.TRACE, "At Position Reference");
        // m_log_atVelocityReference = log.booleanLogger(Level.TRACE, "At Velocity
        // Reference");

        m_log_PositionError = log.doubleLogger(Level.TRACE, "Position Error ");
        // m_log_VelocityError = log.doubleLogger(Level.TRACE, "Velocity Error ");

        m_kPCart = kPCart;
        m_kPTheta = kPTheta;
        m_kPCartV = kPCartV;
        m_kPThetaV = kPThetaV;

        m_xTolerance = xTolerance;
        m_thetaTolerance = thetaTolerance;
        m_xDotTolerance = xDotTolerance;
        // m_omegaTolerance = omegaTolerance;
    }

    @Override
    public GlobalVelocityR3 calculate(
            ModelR3 measurement,
            ModelR3 currentReference,
            ControlR3 nextReference) {
        if (DEBUG) {
            System.out.printf("measurement %s current %s next %s\n", measurement, currentReference, nextReference);
        }
        m_log_measurement.log(() -> measurement);
        m_log_currentReference.log(() -> currentReference);
        m_log_nextReference.log(() -> nextReference);
        GlobalVelocityR3 u_FF = feedforward(nextReference);
        GlobalVelocityR3 u_FB = fullFeedback(measurement, currentReference);
        if (DEBUG) {
            System.out.printf("ff %s fb %s\n", u_FF, u_FB);
        }
        return u_FF.plus(u_FB);
    }

    @Override
    public boolean atReference() {
        return m_atReference;
    }

    @Override
    public void reset() {
        //
    }

    ///////////////////////////////////////////////
    //
    // package-private for testing

    GlobalVelocityR3 feedforward(ControlR3 nextReference) {
        m_log_u_FF.log(() -> nextReference.velocity());
        return nextReference.velocity();
    }

    GlobalVelocityR3 fullFeedback(ModelR3 measurement, ModelR3 currentReference) {
        // atReference is updated below
        m_atReference = true;
        GlobalVelocityR3 u_XFB = positionFeedback(measurement, currentReference);
        GlobalVelocityR3 u_VFB = velocityFeedback(measurement, currentReference);
        return u_XFB.plus(u_VFB);
    }

    GlobalVelocityR3 positionFeedback(ModelR3 measurement, ModelR3 currentReference) {
        // wraps heading
        GlobalDeltaR3 positionError = positionError(measurement, currentReference);
        m_atReference &= positionError.getTranslation().getNorm() < m_xTolerance
                && Math.abs(positionError.getRotation().getRadians()) < m_thetaTolerance;

        GlobalVelocityR3 u_FB = new GlobalVelocityR3(
                m_kPCart * positionError.getX(),
                m_kPCart * positionError.getY(),
                m_kPTheta * positionError.getRotation().getRadians());

        m_log_atPositionReference.log(() -> positionError.getTranslation().getNorm() < m_xTolerance
                && Math.abs(positionError.getRotation().getRadians()) < m_thetaTolerance);
        // System.out.println("THeta Tolerance" +
        // (Math.abs(positionError.getRotation().getRadians()) < m_thetaTolerance));
        // System.out.println("Position Tolerance" +
        // (positionError.getTranslation().getNorm() < m_xTolerance));
        m_log_PositionError.log(() -> positionError.getTranslation().getNorm());
        m_log_u_FB.log(() -> u_FB);
        return u_FB;
    }

    GlobalVelocityR3 velocityFeedback(ModelR3 currentPose, ModelR3 currentReference) {
        final GlobalVelocityR3 velocityError = velocityError(currentPose, currentReference);
        m_atReference &= velocityError.norm() < m_xDotTolerance;
        // && Math.abs(velocityError.angle().orElse(Rotation2d.kZero).getRadians()) <
        // m_omegaTolerance;

        final GlobalVelocityR3 u_VFB = new GlobalVelocityR3(
                m_kPCartV * velocityError.x(),
                m_kPCartV * velocityError.y(),
                m_kPThetaV * velocityError.theta());
        m_log_u_VFB.log(() -> u_VFB);
        // System.out.println("Omega Tolerance" +
        // (Math.abs(velocityError.angle().orElse(Rotation2d.kZero).getRadians()) <
        // m_omegaTolerance));

        // System.out.println("Velocity Tolerance" + (velocityError.norm() <
        // m_xTolerance));

        return u_VFB;
    }

    GlobalDeltaR3 positionError(ModelR3 measurement, ModelR3 currentReference) {
        GlobalDeltaR3 positionError = GlobalDeltaR3.delta(measurement.pose(), currentReference.pose());
        m_log_position_error.log(() -> positionError);
        return positionError;
    }

    GlobalVelocityR3 velocityError(ModelR3 measurement, ModelR3 currentReference) {
        GlobalVelocityR3 velocityError = currentReference.minus(measurement).velocity();
        m_log_velocity_error.log(() -> velocityError);
        return velocityError;
    }

}

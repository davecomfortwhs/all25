package org.team100.lib.motion.swerve.kinodynamics.limiter;

import java.util.function.DoubleSupplier;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.GlobalVelocityR3Logger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;

/**
 * Makes drivetrain input feasible.
 * 
 * Keeps the current setpoint, to avoid round-tripping through the pose
 * estimator. Remember to update the setpoint!
 */
public class SwerveLimiter {
    private static final boolean DEBUG = false;

    private final DoubleLogger m_log_norm;
    private final DoubleLogger m_log_normIn;

    private final GlobalVelocityR3Logger m_log_next;

    private final FieldRelativeVelocityLimiter m_velocityLimiter;
    private final FieldRelativeCapsizeLimiter m_capsizeLimiter;
    private final FieldRelativeAccelerationLimiter m_accelerationLimiter;
    private final SwerveDeadband m_deadband;
    // Velocity expected at the current time, i.e. the previous time step's desire.
    private GlobalVelocityR3 m_current;

    public SwerveLimiter(LoggerFactory parent, SwerveKinodynamics dynamics, DoubleSupplier voltage) {
        LoggerFactory child = parent.type(this);
        m_log_norm = child.doubleLogger(Level.TRACE, "norm");
        m_log_normIn = child.doubleLogger(Level.TRACE, "norm in");
        m_log_next = child.globalVelocityR3Logger(Level.TRACE, "next");

        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(child, dynamics, voltage);
        m_velocityLimiter = new FieldRelativeVelocityLimiter(child, limit);
        m_capsizeLimiter = new FieldRelativeCapsizeLimiter(child, dynamics);

        // Use the absolute maximum acceleration.
        final double cartesianScale = 1.0;
        // Use much less than the maximum rotational acceleration.
        // Rotating fast can be upsetting.
        final double alphaScale = 0.2;
        m_accelerationLimiter = new FieldRelativeAccelerationLimiter(child, dynamics, cartesianScale, alphaScale);

        m_deadband = new SwerveDeadband(child);
    }

    /**
     * Find a feasible setpoint in the direction of the target, and remember it for
     * next time.
     */
    public GlobalVelocityR3 apply(GlobalVelocityR3 nextReference) {
        m_log_next.log(() -> nextReference);
        m_log_normIn.log(nextReference::norm);
        if (DEBUG) {
            System.out.printf("nextReference %s\n", nextReference);
        }
        if (m_current == null)
            m_current = nextReference;

        // First, limit the goal to a feasible velocity.
        GlobalVelocityR3 result = m_velocityLimiter.apply(nextReference);
        if (DEBUG) {
            System.out.printf("velocity limited %s\n", result);
        }

        // then limit acceleration towards that goal to avoid capsize
        result = m_capsizeLimiter.apply(m_current, result);
        if (DEBUG) {
            System.out.printf("capsize limited %s\n", result);
        }

        // Finally, limit acceleration further, using motor physics.
        result = m_accelerationLimiter.apply(m_current, result);
        if (DEBUG) {
            System.out.printf("accel limited %s\n", result);
        }

        // Ignore very small inputs.
        if (Experiments.instance.enabled(Experiment.SwerveDeadband)) {
            result = m_deadband.apply(result);
        }

        updateSetpoint(result);

        if (DEBUG) {
            System.out.printf("result %s\n", result);
        }
        m_log_norm.log(result::norm);

        return result;
    }

    /**
     * Set the current setpoint to the current velocity measurement.
     * This is required to make resumption of manual control smooth.
     */
    public void updateSetpoint(GlobalVelocityR3 setpoint) {
        m_current = setpoint;
    }

}

package org.team100.lib.motion.swerve.kinodynamics.limiter;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GlobalAccelerationR3;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.GlobalAccelerationR3Logger;
import org.team100.lib.logging.LoggerFactory.GlobalVelocityR3Logger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;

/**
 * Limits acceleration to avoid tipping over.
 */
public class FieldRelativeCapsizeLimiter {
    private static final boolean DEBUG = false;

    private final DoubleLogger m_log_scale;
    private final GlobalAccelerationR3Logger m_log_accel;
    private final GlobalVelocityR3Logger m_log_prev;
    private final GlobalVelocityR3Logger m_log_target;

    private final SwerveKinodynamics limits;

    public FieldRelativeCapsizeLimiter(
            LoggerFactory parent,
            SwerveKinodynamics m_limits) {
        LoggerFactory child = parent.type(this);
        m_log_scale = child.doubleLogger(Level.TRACE, "scale");
        m_log_accel = child.globalAccelerationR3Logger(Level.TRACE, "accel");
        m_log_prev = child.globalVelocityR3Logger(Level.TRACE, "prev");
        m_log_target = child.globalVelocityR3Logger(Level.TRACE, "target");
        limits = m_limits;
    }

    public GlobalVelocityR3 apply(
            GlobalVelocityR3 prev,
            GlobalVelocityR3 target) {
        m_log_prev.log(() -> prev);
        m_log_target.log(() -> target);
        // Acceleration required to achieve the target.
        GlobalAccelerationR3 accel = target.accel(
                prev,
                TimedRobot100.LOOP_PERIOD_S);
        m_log_accel.log(() -> accel);
        double a = accel.norm();
        if (a < 1e-6) {
            // Zero acceleration.
            a = 0;
        }
        double scale = scale(a);
        m_log_scale.log(() -> scale);
        GlobalVelocityR3 result = prev.plus(accel.times(scale).integrate(TimedRobot100.LOOP_PERIOD_S));
        if (DEBUG) {
            System.out.printf("FieldRelativeCapsizeLimiter prev %s target %s accel %s scale %5.2f result %s\n",
                    prev, target, accel, scale, result);
        }
        return result;
    }

    double scale(double a) {
        return Math.min(1, limits.getMaxCapsizeAccelM_S2() / a);
    }
}

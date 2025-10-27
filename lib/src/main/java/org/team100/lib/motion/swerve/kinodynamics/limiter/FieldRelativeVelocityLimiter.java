package org.team100.lib.motion.swerve.kinodynamics.limiter;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;

/**
 * Return feasible velocity, using a simple worst-case model (diagonal course).
 */
public class FieldRelativeVelocityLimiter {
    private static final boolean DEBUG = false;

    private final DoubleLogger m_log_scale;
    private final BatterySagSpeedLimit m_limits;

    public FieldRelativeVelocityLimiter(
            LoggerFactory parent,
            BatterySagSpeedLimit limit) {
        LoggerFactory child = parent.type(this);
        m_log_scale = child.doubleLogger(Level.TRACE, "scale");
        m_limits = limit;
    }

    public GlobalVelocityR3 apply(GlobalVelocityR3 target) {
        if (Experiments.instance.enabled(Experiment.LimitsPreferRotation))
            return preferRotation(target);
        return proportional(target);
    }

    /** Maintain translation and rotation proportionality. */
    GlobalVelocityR3 proportional(GlobalVelocityR3 target) {
        if (DEBUG) {
            System.out.printf("proportional %s\n", target);
        }
        final double maxV = m_limits.getMaxDriveVelocityM_S();
        final double maxOmega = m_limits.getMaxAngleSpeedRad_S();
        double xySpeed = target.norm();

        // this could be negative if xySpeed is too high
        double omegaForSpeed = maxOmega * (1 - xySpeed / maxV);
        boolean feasible = Math.abs(target.theta()) <= omegaForSpeed;
        if (feasible) {
            // omega + xyspeed is feasible
            m_log_scale.log(() -> 1.0);
            if (DEBUG) {
                System.out.printf("feasible %s\n", target);
            }
            return target;
        }

        if (xySpeed < 1e-12) {
            return spinOnly(target, maxOmega);
        }

        if (Math.abs(target.theta()) < 1e-12) {
            return translateOnly(target, maxV, xySpeed);
        }

        return proportional(target, maxV, maxOmega, xySpeed);
    }

    /** Both rotation and translation, scale proportionally. */
    private GlobalVelocityR3 proportional(
            GlobalVelocityR3 target,
            final double maxV,
            final double maxOmega,
            double xySpeed) {
        double v = maxOmega * xySpeed * maxV / (maxOmega * xySpeed + Math.abs(target.theta()) * maxV);
        double scale = v / xySpeed;
        m_log_scale.log(() -> scale);
        if (DEBUG) {
            System.out.printf("FieldRelativeVelocityLimiter proportional scale %.5f\n", scale);
        }
        return new GlobalVelocityR3(
                scale * target.x(),
                scale * target.y(),
                scale * target.theta());
    }

    /** No rotation at all, so use maxV. */
    private GlobalVelocityR3 translateOnly(GlobalVelocityR3 target, double maxV, double xySpeed) {
        double xyAngle = Math.atan2(target.y(), target.x());
        double scale = Math.abs(maxV / xySpeed);
        m_log_scale.log(() -> scale);
        if (DEBUG) {
            System.out.printf("max v %s\n", target);
        }
        return new GlobalVelocityR3(
                maxV * Math.cos(xyAngle),
                maxV * Math.sin(xyAngle),
                0);
    }

    /** Spinning in place, faster than is possible, so use maxOmega. */
    private GlobalVelocityR3 spinOnly(GlobalVelocityR3 target, double maxOmega) {
        double scale = Math.abs(maxOmega / target.theta());
        m_log_scale.log(() -> scale);
        if (DEBUG) {
            System.out.printf("max omega %s\n", target);
        }
        return new GlobalVelocityR3(
                0,
                0,
                Math.signum(target.theta()) * maxOmega);
    }

    /** Scales translation to accommodate the rotation. */
    GlobalVelocityR3 preferRotation(GlobalVelocityR3 speeds) {
        double omegaRatio = Math.min(1, speeds.theta() / m_limits.getMaxAngleSpeedRad_S());
        double xySpeed = speeds.norm();
        double maxV = m_limits.getMaxDriveVelocityM_S();
        double xyRatio = Math.min(1, xySpeed / maxV);
        double ratio = Math.min(1 - omegaRatio, xyRatio);

        // just for logging
        double scale = ratio / xyRatio;
        m_log_scale.log(() -> scale);

        double xyAngle = Math.atan2(speeds.y(), speeds.x());

        if (DEBUG) {
            System.out.printf("FieldRelativeVelocityLimiter rotation ratio %.5f\n", ratio);
        }

        return new GlobalVelocityR3(
                ratio * maxV * Math.cos(xyAngle),
                ratio * maxV * Math.sin(xyAngle),
                speeds.theta());
    }

}

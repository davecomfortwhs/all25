package org.team100.lib.motion.swerve.kinodynamics.limiter;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GlobalAccelerationR3;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.util.Math100;

public class SwerveUtil {
    private static final boolean DEBUG = false;

    /**
     * At low speed, accel is limited by the current limiters.
     * At high speed, accel is limited by back EMF.
     * Deceleration limits are different: back EMF is helping in that case, but we
     * just return the maximum, which is less than the real maximum.
     * 
     * @see SwerveDriveDynamicsConstraint.getMinMaxAcceleration().
     */
    public static double getAccelLimit(
            SwerveKinodynamics m_limits,
            double vScale,
            double aScale,
            GlobalVelocityR3 prev,
            GlobalVelocityR3 desired) {
        if (isAccel(prev, desired)) {
            return minAccel(m_limits, vScale, aScale, prev.norm());
        }
        return aScale * m_limits.getMaxDriveDecelerationM_S2();
    }

    /**
     * At low speed, accel is limited by the current limiters.
     * At high speed, accel is limited by back EMF.
     * Note at full speed this can return zero.
     */
    public static double minAccel(SwerveKinodynamics m_limits, double vScale, double aScale, double velocity) {
        double speedFraction = Math100.limit(velocity / (vScale * m_limits.getMaxDriveVelocityM_S()), 0, 1);
        double backEmfLimit = 1 - speedFraction;
        double backEmfLimitedAcceleration = backEmfLimit * aScale * m_limits.getStallAccelerationM_S2();
        double currentLimitedAcceleration = aScale * m_limits.getMaxDriveAccelerationM_S2();
        if (DEBUG) {
            System.out.printf("speedFraction %5.2f backEmfLimitedAcceleration %5.2f currentLimitedAcceleration %5.2f\n",
                    speedFraction, backEmfLimitedAcceleration, currentLimitedAcceleration);
        }
        return Math.min(backEmfLimitedAcceleration, currentLimitedAcceleration);
    }

    /**
     * Find the desired dv. Project it on to the previous v: if the projection is
     * positive, we're accelerating, otherwise decelerating.
     * 
     * This correctly captures sharp turns as decelerations; simply comparing the
     * magnitudes of initial and final velocities is not correct.
     */
    static boolean isAccel(GlobalVelocityR3 prev,
            GlobalVelocityR3 target) {
        GlobalAccelerationR3 accel = target.accel(prev, TimedRobot100.LOOP_PERIOD_S);
        double dot = prev.x() * accel.x() + prev.y() * accel.y();
        return dot >= 0;
    }

    private SwerveUtil() {
        //
    }
}

package org.team100.lib.trajectory.timing;

import org.team100.lib.geometry.Pose2dWithMotion;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Limits torque due to acceleration along the path.
 * 
 * This is for the Calgames PRR mechanism, so the center of rotation is the
 * origin (middle of the robot on the floor, if it were calibrated; at
 * the moment, it's the shoulder axis when at rest.)
 * 
 * F = ma for torque is the cross product:
 * 
 * \tau = m(\vec{r} \times \vec{a})
 * 
 * We know the direction of \vec{a}, so use a scalar and a unit vector:
 * 
 * \tau = ma(\vec{r} \times \vec{u})
 * 
 * So
 * 
 * a = \tau / (m (\vec{r} \times \vec{u}))
 * 
 * For capsize torque, mass is roughly 50 kg, and the lever from COM to wheel is
 * about 0.3 m, so about 150 Nm. Experimentally, we find that we want much lower
 * limits than this, to avoid coming anywhere near the limit.
 * 
 * See https://en.wikipedia.org/wiki/Angular_acceleration
 */
public class TorqueConstraint implements TimingConstraint {
    private static final boolean DEBUG = false;
    /** Approximate mass of end effector in kg. */
    private static final double M = 6.0;
    /** Maximum allowed torque in Nm. */
    private final double m_maxTorque;

    /**
     * @param maxTorque max torque in Nm
     */
    public TorqueConstraint(double maxTorque) {
        m_maxTorque = maxTorque;
    }

    @Override
    public NonNegativeDouble getMaxVelocity(Pose2dWithMotion state) {
        // Do not constrain velocity.
        return new NonNegativeDouble(Double.POSITIVE_INFINITY);
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(
            Pose2dWithMotion state, double velocityM_S) {
        // If there's no course, assume the worst case.
        Rotation2d course = state.getCourse().orElse(Rotation2d.kCCW_90deg);
        // acceleration unit vector
        Translation2d u = new Translation2d(1.0, course);
        Translation2d r = state.getPose().getTranslation();
        double cross = r.getX() * u.getY() - r.getY() * u.getX();
        double a = Math.abs(m_maxTorque / (M * cross));
        if (DEBUG) {
            System.out.printf("Torque Constraint a: %6.3f p: %s r: %6.3f course: %6.3f\n",
                    a, state.getPose(), r.getNorm(), course.getRadians());
        }
        return new MinMaxAcceleration(-a, a);
    }
}

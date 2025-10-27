package org.team100.lib.trajectory.timing;

import java.util.Optional;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.tuning.Mutable;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Mecanum drive has a diamond-shaped velocity envelope. If the x and y
 * directions responded the same (they don't), it would be a square.
 * 
 * This ignores the interaction with rotation.
 */
public class DiamondConstraint implements TimingConstraint {
    /** Max velocity ahead */
    private final Mutable m_maxVelocityX;
    /** Max velocity to the side */
    private final Mutable m_maxVelocityY;
    private final Mutable m_maxAccel;

    /**
     * @param parent log
     * @param maxVX  max velocity straight ahead, typically higher
     * @param maxVY  max velocity sideways, typically lower
     * @param maxA   accel
     */
    public DiamondConstraint(LoggerFactory parent, double maxVX, double maxVY, double maxA) {
        LoggerFactory log = parent.type(this);
        m_maxVelocityX = new Mutable(log, "maxVX", maxVX);
        m_maxVelocityY = new Mutable(log, "maxVY", maxVY);
        m_maxAccel = new Mutable(log, "maxA", maxA);
    }

    @Override
    public NonNegativeDouble getMaxVelocity(Pose2dWithMotion state) {
        Optional<Rotation2d> courseOpt = state.getCourse();
        if (courseOpt.isEmpty()) {
            // robot is stationary. turning in place?
            return new NonNegativeDouble(m_maxVelocityY.getAsDouble());
        }
        Rotation2d course = courseOpt.get();
        Rotation2d heading = state.getHeading();
        Rotation2d strafe = course.minus(heading);
        // a rhombus is a superellipse with exponent 1
        // https://en.wikipedia.org/wiki/Superellipse
        double a = m_maxVelocityX.getAsDouble();
        double b = m_maxVelocityY.getAsDouble();
        double r = 1 / (Math.abs(strafe.getCos() / a) + Math.abs(strafe.getSin() / b));
        return new NonNegativeDouble(r);
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state, double velocityM_S) {
        return new MinMaxAcceleration(-m_maxAccel.getAsDouble(), m_maxAccel.getAsDouble());
    }

}

package org.team100.lib.trajectory.timing;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.tuning.Mutable;

/** Trivial constraint for testing. */
public class ConstantConstraint implements TimingConstraint {
    private final Mutable m_maxVelocity;
    private final Mutable m_maxAccel;

    public ConstantConstraint(LoggerFactory parent, double maxV, double maxA) {
        LoggerFactory log = parent.type(this);
        m_maxVelocity = new Mutable(log, "maxV", maxV);
        m_maxAccel = new Mutable(log, "maxA", maxA);
    }

    public ConstantConstraint(LoggerFactory log, double vScale, double aScale, SwerveKinodynamics limits) {
        this(log, vScale * limits.getMaxDriveVelocityM_S(), aScale * limits.getMaxDriveAccelerationM_S2());
    }

    @Override
    public NonNegativeDouble getMaxVelocity(Pose2dWithMotion state) {
        return new NonNegativeDouble(m_maxVelocity.getAsDouble());
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state, double velocityM_S) {
        return new MinMaxAcceleration(-m_maxAccel.getAsDouble(), m_maxAccel.getAsDouble());
    }

}

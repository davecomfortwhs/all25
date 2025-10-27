package org.team100.lib.commands.swerve.manual;

import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ChassisSpeedsLogger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.state.ModelR3;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Transform manual input into ChassisSpeeds.
 * 
 * The twist components, x, y, and theta, are mapped directly to the
 * corresponding ChassisSpeeds components (and scaled).
 */
public class ManualChassisSpeeds implements ChassisSpeedDriver {
    private final SwerveKinodynamics m_swerveKinodynamics;
    // LOGGERS
    private final ChassisSpeedsLogger m_log_chassis_speeds;

    public ManualChassisSpeeds(LoggerFactory parent, SwerveKinodynamics swerveKinodynamics) {
        LoggerFactory child = parent.type(this);
        m_log_chassis_speeds = child.chassisSpeedsLogger(Level.TRACE, "chassis speeds");
        m_swerveKinodynamics = swerveKinodynamics;
    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds.
     */
    @Override
    public ChassisSpeeds apply(ModelR3 state, Velocity input) {
        // clip the input to the unit circle
        final Velocity clipped = input.clip(1.0);
        // scale to max in both translation and rotation

        final ChassisSpeeds scaled = scaleChassisSpeeds(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        m_log_chassis_speeds.log(() -> scaled);
        return scaled;
    }

    /**
     * Scales driver input to robot-relative velocity.
     * 
     * This makes no attempt to address infeasibilty, it just multiplies.
     * 
     * @param twist    [-1,1]
     * @param maxSpeed meters per second
     * @param maxRot   radians per second
     * @return meters and rad per second as specified by speed limits
     */
    public static ChassisSpeeds scaleChassisSpeeds(Velocity twist, double maxSpeed, double maxRot) {
        return new ChassisSpeeds(
                maxSpeed * MathUtil.clamp(twist.x(), -1, 1),
                maxSpeed * MathUtil.clamp(twist.y(), -1, 1),
                maxRot * MathUtil.clamp(twist.theta(), -1, 1));
    }

    public void reset(ModelR3 p) {
        //
    }
}

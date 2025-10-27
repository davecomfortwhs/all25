package org.team100.lib.motion.mecanum;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.tuning.Mutable;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;

/**
 * Includes simple correction factors to account for wheel slip.
 * 
 * The slip varies depending on course -- more slip when moving sideways, less
 * slip when moving ahead.
 * 
 * Factor greater than one means the wheels will go faster than they would in
 * the zero-slip case, in order to make the actual velocity what is requested.
 */
public class MecanumKinematics100 {
    public record Slip(double kx, double ky, double ktheta) {
    }

    private final MecanumDriveKinematics m_kinematics;
    private final Mutable m_kx;
    private final Mutable m_ky;
    private final Mutable m_ktheta;

    public MecanumKinematics100(
            LoggerFactory parent, Slip slip,
            Translation2d fl, Translation2d fr, Translation2d rl, Translation2d rr) {
        m_kinematics = new MecanumDriveKinematics(fl, fr, rl, rr);
        LoggerFactory log = parent.type(this);
        m_kx = new Mutable(log, "Slip kx", slip.kx);
        m_ky = new Mutable(log, "Slip ky", slip.ky);
        m_ktheta = new Mutable(log, "Slip ktheta", slip.ktheta);
    }

    public MecanumDriveWheelSpeeds toWheelSpeeds(ChassisSpeeds actual) {
        // Slipping wheels need to go faster than the actual speed.
        ChassisSpeeds slipping = new ChassisSpeeds(
                m_kx.getAsDouble() * actual.vxMetersPerSecond,
                m_ky.getAsDouble() * actual.vyMetersPerSecond,
                m_ktheta.getAsDouble() * actual.omegaRadiansPerSecond);
        return m_kinematics.toWheelSpeeds(slipping);
    }

    public Twist2d toTwist2d(MecanumDriveWheelPositions start, MecanumDriveWheelPositions end) {
        Twist2d slipping = m_kinematics.toTwist2d(start, end);
        // Actual speed is slower than the slipping wheels would indicate.
        return new Twist2d(
                slipping.dx / m_kx.getAsDouble(),
                slipping.dy / m_ky.getAsDouble(),
                slipping.dtheta / m_ktheta.getAsDouble());
    }

}

package org.team100.lib.motion.swerve;

import org.team100.lib.localization.AprilTagRobotLocalizer;
import org.team100.lib.localization.FreshSwerveEstimate;
import org.team100.lib.localization.OdometryUpdater;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.motion.swerve.module.SwerveModuleCollection;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Pull together some of the drivetrain's dependencies so they don't pollute
 * Robot.java.
 */
public class SwerveDriveFactory {

    public static SwerveDriveSubsystem get(
            LoggerFactory fieldLogger,
            LoggerFactory driveLog,
            SwerveKinodynamics m_swerveKinodynamics,
            AprilTagRobotLocalizer localizer,
            OdometryUpdater odometryUpdater,
            SwerveHistory history,
            SwerveModuleCollection m_modules) {

        FreshSwerveEstimate estimate = new FreshSwerveEstimate(
                localizer,
                odometryUpdater,
                history);
        SwerveLocal swerveLocal = new SwerveLocal(
                driveLog,
                m_swerveKinodynamics,
                m_modules);
        SwerveLimiter limiter = new SwerveLimiter(
                driveLog,
                m_swerveKinodynamics,
                RobotController::getBatteryVoltage);
        return new SwerveDriveSubsystem(
                fieldLogger,
                driveLog,
                m_swerveKinodynamics,
                odometryUpdater,
                estimate,
                swerveLocal,
                limiter);
    }

}

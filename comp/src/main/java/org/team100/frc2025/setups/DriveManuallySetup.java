package org.team100.frc2025.setups;

import org.team100.frc2025.Swerve.ManualWithBargeAssist;
import org.team100.frc2025.Swerve.ManualWithProfiledReefLock;
import org.team100.lib.commands.swerve.manual.DriveManually;
import org.team100.lib.commands.swerve.manual.ManualChassisSpeeds;
import org.team100.lib.commands.swerve.manual.ManualFieldRelativeSpeeds;
import org.team100.lib.commands.swerve.manual.ManualWithFullStateHeading;
import org.team100.lib.commands.swerve.manual.ManualWithProfiledHeading;
import org.team100.lib.commands.swerve.manual.ManualWithTargetLock;
import org.team100.lib.commands.swerve.manual.SimpleManualModuleStates;
import org.team100.lib.controller.r1.Feedback100;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.localization.AprilTagRobotLocalizer;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.swerve.SwerveDriveSubsystem;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.limiter.SwerveLimiter;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * This is the set of manual driving modes we usually support,
 * but we're not using these in 2025, we're using DriveManuallySimple instead.
 */
public class DriveManuallySetup {

    public static void setup(
            LoggerFactory comLog,
            FieldLogger.Log fieldLog,
            DriverXboxControl driverControl,
            AprilTagRobotLocalizer localizer,
            SwerveDriveSubsystem drive,
            SwerveLimiter limiter,
            SwerveKinodynamics swerveKinodynamics,
            Feedback100 thetaFeedback) {
        final DriveManually driveManually = new DriveManually(
                driverControl::velocity,
                localizer::setHeedRadiusM,
                drive,
                limiter);

        final LoggerFactory manLog = comLog.type(driveManually);

        driveManually.register("MODULE_STATE", false,
                new SimpleManualModuleStates(manLog, swerveKinodynamics));

        driveManually.register("ROBOT_RELATIVE_CHASSIS_SPEED", false,
                new ManualChassisSpeeds(manLog, swerveKinodynamics));

        driveManually.register("FIELD_RELATIVE_TWIST", false,
                new ManualFieldRelativeSpeeds(manLog, swerveKinodynamics));

        driveManually.register("SNAPS_PROFILED", true,
                new ManualWithProfiledHeading(
                        manLog,
                        swerveKinodynamics,
                        driverControl::pov,
                        thetaFeedback));

        driveManually.register("SNAPS_FULL_STATE", true,
                new ManualWithFullStateHeading(
                        manLog,
                        swerveKinodynamics,
                        driverControl::pov,
                        new double[] {
                                5,
                                0.35
                        }));

        /**
         * in reality, the target would come from some designator, e.g. buttons or
         * camera or whatever.
         */
        driveManually.register("LOCKED", false,
                new ManualWithTargetLock(
                        fieldLog,
                        manLog,
                        swerveKinodynamics,
                        () -> new Translation2d(6, 4),
                        thetaFeedback));

        driveManually.register("BARGE ASSIST", false,
                new ManualWithBargeAssist(
                        manLog,
                        swerveKinodynamics,
                        driverControl::pov,
                        thetaFeedback,
                        drive::getPose));

        driveManually.register("REEF LOCK", false,
                new ManualWithProfiledReefLock(
                        manLog,
                        swerveKinodynamics,
                        driverControl::leftTrigger,
                        thetaFeedback,
                        () -> drive.getPose().getTranslation()));
    }
}

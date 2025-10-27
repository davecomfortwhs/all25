package org.team100.lib.commands.swerve;

import org.team100.lib.motion.swerve.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Set the drivetrain pose to the specified pose.
 * 
 * This is useful to reset where "zero" is.
 * 
 * If you just want to reset rotation, use SetRotation.
 */
public class ResetPose extends InstantCommand {

    public ResetPose(SwerveDriveSubsystem drive, Pose2d pose) {
        super(() -> {
            drive.resetPose(pose);
            drive.stop();
        }, drive);
    }
}
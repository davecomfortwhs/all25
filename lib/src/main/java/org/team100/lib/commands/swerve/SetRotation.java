package org.team100.lib.commands.swerve;

import org.team100.lib.motion.swerve.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Set the rotation of the robot pose to the specified rotation.
 * 
 * This is good for resetting the "zero," in cases where the robot isn't
 * actually facing zero (e.g. 180)
 */
public class SetRotation extends InstantCommand {

    public SetRotation(
            SwerveDriveSubsystem drive,
            Rotation2d rotation) {
        super(() -> drive.resetPose(
                new Pose2d(drive.getPose().getTranslation(), rotation)),
                drive);
    }
}

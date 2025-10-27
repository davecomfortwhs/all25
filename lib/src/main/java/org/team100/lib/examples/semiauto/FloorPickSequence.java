package org.team100.lib.examples.semiauto;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.commands.r3.DriveToTranslationWithRelativeBearing;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.motion.swerve.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.targeting.Targets;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Use a sequence of commands to drive nearby a target with the intake facing
 * it, and then sweep the intake over the target.
 */
public class FloorPickSequence {

    // landing point distance to target
    private static final double DISTANCE = 1.0;
    private static final double PICKOFFSET = 0.193;  // pick is from the back
    private static final Rotation2d RELATIVE_BEARING = Rotation2d.k180deg;

    public static Command get(
            FieldLogger.Log fieldLog,
            SwerveDriveSubsystem drive,
            Targets targets,
            ControllerR3 controller,
            HolonomicProfile profile) {
        Supplier<Optional<Translation2d>> target = () -> Optional.of(targets.getClosestTarget().get().plus(new Translation2d(0,PICKOFFSET
        )));
        Supplier<Optional<Translation2d>> runway = () -> {
            Optional<Translation2d> t = target.get();
            if (t.isEmpty())
                return t;
            Translation2d goal = t.get();
            Rotation2d course = goal.minus(drive.getPose().getTranslation()).getAngle();
            Translation2d landing = goal.minus(new Translation2d(DISTANCE, course));
            return Optional.of(landing);
        };

        DriveToTranslationWithRelativeBearing toRunway = new DriveToTranslationWithRelativeBearing(
                fieldLog, runway, drive, controller, profile, RELATIVE_BEARING);
        DriveToTranslationWithRelativeBearing toTarget = new DriveToTranslationWithRelativeBearing(
                fieldLog, target, drive, controller, profile, RELATIVE_BEARING);

        return sequence(
                toRunway.until(toRunway::thetaAligned),
                toTarget.until(toTarget::isDone));
    }

}

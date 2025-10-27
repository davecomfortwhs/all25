package org.team100.frc2025.CommandGroups.ScoreSmart;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.function.Supplier;

import org.team100.frc2025.CalgamesArm.CalgamesMech;
import org.team100.frc2025.grip.Manipulator;
import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.commands.r3.DriveToPoseWithProfile;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.swerve.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ScoreL3Smart {
    public static Command get(
            LoggerFactory logger,
            CalgamesMech mech,
            Manipulator manipulator,
            ControllerR3 controller,
            HolonomicProfile profile,
            SwerveDriveSubsystem m_drive,
            Supplier<Pose2d> goal) {
        DriveToPoseWithProfile toReef = new DriveToPoseWithProfile(
                logger, m_drive, controller, profile, goal);
        MoveAndHold toL3 = mech.homeToL3();
        Command eject = manipulator.centerEject().withTimeout(0.5);
        return sequence(
                parallel(
                        toReef,
                        waitUntil(() -> toReef.toGo() < 1)
                                .andThen(toL3),
                        waitUntil(() -> (toReef.isDone() && toL3.isDone()))
                                .andThen(eject) //
                ).until(() -> (toReef.isDone() && toL3.isDone() && eject.isFinished())),
                mech.l3ToHome() //
        );
    }
}

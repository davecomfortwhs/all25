package org.team100.frc2025.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.commands.r3.DriveToPoseWithProfile;
import org.team100.lib.commands.r3.DriveToTranslationFacingWithProfile;
import org.team100.lib.commands.r3.DriveWithTrajectoryFunction;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.controller.r3.FullStateControllerR3;
import org.team100.lib.field.FieldConstants;
import org.team100.lib.field.FieldConstants.ReefPoint;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.trajectory.TrajectoryPlanner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

public class LolipopAuto {
    private static final double HEED_RADIUS_M = 3;
    private final LoggerFactory m_logger;
    private final Machinery m_machinery;
    private final HolonomicProfile m_autoProfile;
    private final FullStateControllerR3 m_autoController;
    private final TrajectoryPlanner m_planner;

    public LolipopAuto(
            LoggerFactory logger,
            Machinery machinery,
            HolonomicProfile autoProfile,
            FullStateControllerR3 autoController,
            TrajectoryPlanner planner) {
        m_logger = logger;
        m_machinery = machinery;
        m_autoProfile = autoProfile;
        m_autoController = autoController;
        m_planner = planner;
    }

    public Command get() {
        // this one uses some curvature
        DriveWithTrajectoryFunction toReefTrajectory = new DriveWithTrajectoryFunction(
                m_machinery.m_drive, m_autoController, m_machinery.m_trajectoryViz,
                (p) -> m_planner.restToRest(List.of(
                        HolonomicPose2d.make(m_machinery.m_drive.getPose(), Math.PI),
                        HolonomicPose2d.make(3, 5, 0, -2))));

        DriveToPoseWithProfile toReefA = new DriveToPoseWithProfile(
                m_logger, m_machinery.m_drive, m_autoController, m_autoProfile,
                () -> FieldConstants.makeGoal(ScoringLevel.L4, ReefPoint.A));

        DriveToTranslationFacingWithProfile toCenterCoral = new DriveToTranslationFacingWithProfile(
                m_logger, m_machinery.m_drive, m_autoController, m_autoProfile,
                () -> FieldConstants.CoralMark.CENTER.value
                        .plus(new Translation2d(0.7, 0)),
                new Rotation2d(Math.PI));

        DriveToPoseWithProfile toReefB = new DriveToPoseWithProfile(
                m_logger, m_machinery.m_drive, m_autoController, m_autoProfile,
                () -> FieldConstants.makeGoal(ScoringLevel.L4, ReefPoint.B));

        DriveToTranslationFacingWithProfile toCoralRight = new DriveToTranslationFacingWithProfile(
                m_logger, m_machinery.m_drive, m_autoController, m_autoProfile,
                () -> FieldConstants.CoralMark.RIGHT.value
                        .plus(new Translation2d(0.7, 0).rotateBy(new Rotation2d(Math.PI / 4))),
                new Rotation2d(Math.PI));

        DriveToPoseWithProfile toReefC = new DriveToPoseWithProfile(
                m_logger, m_machinery.m_drive, m_autoController, m_autoProfile,
                () -> FieldConstants.makeGoal(ScoringLevel.L4, ReefPoint.C));

        MoveAndHold toL4 = m_machinery.m_mech.homeToL4();
        MoveAndHold toL4second = m_machinery.m_mech.homeToL4();
        MoveAndHold toL4third = m_machinery.m_mech.homeToL4();
        ParallelRaceGroup eject = m_machinery.m_manipulator.centerEject().withTimeout(0.3);
        ParallelRaceGroup ejectsecond = m_machinery.m_manipulator.centerEject().withTimeout(0.3);
        ParallelRaceGroup ejectthird = m_machinery.m_manipulator.centerEject().withTimeout(0.3);
        return sequence(
                toReefTrajectory.until(toReefTrajectory::isDone),
                parallel(
                        runOnce(() -> m_machinery.m_localizer.setHeedRadiusM(HEED_RADIUS_M)),
                        toReefA,
                        m_machinery.m_mech.profileHomeAndThenRest().until(toReefA::isDone).andThen(toL4),
                        waitUntil(() -> toReefA.isDone() && toL4.isDone())
                                .andThen(eject))
                        .until(() -> (toReefA.isDone() && toL4.isDone() && eject.isFinished())),
                parallel(
                        toCenterCoral,
                        m_machinery.m_mech.pickWithProfile(),
                        m_machinery.m_manipulator.centerIntake()).until(m_machinery.m_manipulator::hasCoral)
                        .withTimeout(3)
                        .andThen(m_machinery.m_manipulator::stop),
                parallel(
                        toReefB,
                        m_machinery.m_mech.profileHomeAndThenRest().until(toReefB::isDone).andThen(toL4second),
                        waitUntil(() -> toReefB.isDone() && toL4second.isDone())
                                .andThen(ejectsecond))
                        .until(() -> (toReefB.isDone() && toL4second.isDone() && ejectsecond.isFinished())),
                parallel(
                        toCoralRight,
                        m_machinery.m_mech.pickWithProfile(),
                        m_machinery.m_manipulator.centerIntake()).until(m_machinery.m_manipulator::hasCoral)
                        .withTimeout(3)
                        .andThen(m_machinery.m_manipulator::stop),
                parallel(
                        toReefC,
                        m_machinery.m_mech.profileHomeAndThenRest().until(toReefC::isDone).andThen(toL4third),
                        waitUntil(() -> toReefC.isDone() && toL4third.isDone())
                                .andThen(ejectthird))
                        .until(() -> (toReefC.isDone() && toL4third.isDone() && ejectthird.isFinished())),
                m_machinery.m_mech.l4ToHome());
    }
}

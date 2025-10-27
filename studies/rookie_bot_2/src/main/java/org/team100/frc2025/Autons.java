package org.team100.frc2025;

import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.commands.r3.DriveToPoseWithProfile;
import org.team100.lib.commands.r3.DriveWithTrajectoryFunction;
import org.team100.lib.commands.r3.FeedforwardOnly;
import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.config.AutonChooser;
import org.team100.lib.controller.r3.ControllerFactoryR3;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mecanum.MecanumDrive100;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.DiamondConstraint;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.YawRateConstraint;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class Autons {
    private static final Pose2d ONE = new Pose2d(4, 1, Rotation2d.kZero);
    private static final Pose2d TWO = new Pose2d(1, 4, Rotation2d.k180deg);
    private static final Pose2d FOUR = new Pose2d(4, 4, Rotation2d.kCCW_90deg);

    private final AutonChooser m_autonChooser;
    private final MecanumDrive100 m_drive;
    private final HolonomicProfile m_profile;
    private final TrajectoryPlanner m_planner;
    private final TrajectoryVisualization m_viz;

    public Autons(
            LoggerFactory log,
            LoggerFactory fieldLogger,
            MecanumDrive100 drive) {
        LoggerFactory autoLog = log.name("Auton");
        m_autonChooser = new AutonChooser();
        m_drive = drive;
        m_profile = HolonomicProfile.wpi(4, 8, 3, 6);
        List<TimingConstraint> constraints = List.of(
                new DiamondConstraint(autoLog, 2, 2, 2),
                new ConstantConstraint(autoLog, 2, 2),
                new YawRateConstraint(autoLog, 1, 1));
        m_planner = new TrajectoryPlanner(constraints);
        m_viz = new TrajectoryVisualization(fieldLogger);

        ControllerR3 controller = ControllerFactoryR3.byIdentity(autoLog);

        MoveAndHold one = new FeedforwardOnly(m_profile, ONE, m_drive);
        m_autonChooser.add("one",
                new AnnotatedCommand(one.until(one::isDone).withName("auto one"), null, null));

        MoveAndHold two = new FeedforwardOnly(m_profile, TWO, m_drive);
        m_autonChooser.add("two",
                new AnnotatedCommand(two.until(two::isDone).withName("auto two"), null, null));

        Command three = m_drive.driveWithGlobalVelocity(
                new GlobalVelocityR3(1.5, 0, 0)).withTimeout(1.0);
        m_autonChooser.add("three",
                new AnnotatedCommand(three.withName("auto three"), null, null));

        MoveAndHold four = new DriveToPoseWithProfile(
                autoLog, drive, controller, m_profile, () -> FOUR);
        m_autonChooser.add("four",
                new AnnotatedCommand(four.until(four::isDone).withName("auto four"), null, null));

        MoveAndHold five = new DriveWithTrajectoryFunction(
                drive, controller, m_viz, this::five);
        m_autonChooser.add("five",
                new AnnotatedCommand(five.until(five::isDone).withName("auto five"),
                        Alliance.Red, Field.START_RED_RIGHT));
    }

    public AnnotatedCommand get() {
        return m_autonChooser.get();
    }

    private Trajectory100 five(Pose2d p) {
        return m_planner.restToRest(List.of(
                HolonomicPose2d.make(p, 0),
                HolonomicPose2d.make(1, 2, Math.PI / 2, Math.PI / 2)));
    }

}

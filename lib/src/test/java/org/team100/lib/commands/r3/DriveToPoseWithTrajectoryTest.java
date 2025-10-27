package org.team100.lib.commands.r3;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.r3.ControllerFactoryR3;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.swerve.Fixtured;
import org.team100.lib.motion.swerve.SwerveDriveSubsystem;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.TimedPose;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.TimingConstraintFactory;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

class DriveToPoseWithTrajectoryTest extends Fixtured implements Timeless {
    public DriveToPoseWithTrajectoryTest() throws IOException {
    }

    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final TrajectoryVisualization viz = new TrajectoryVisualization(logger);

    SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
    List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood(logger);
    TrajectoryPlanner planner = new TrajectoryPlanner(constraints);

    @Test
    void testSimple() {
        Pose2d goal = Pose2d.kZero;
        SwerveDriveSubsystem drive = fixture.drive;

        ControllerR3 controller = ControllerFactoryR3.test(logger);
        DriveToPoseWithTrajectory command = new DriveToPoseWithTrajectory(
                () -> goal,
                drive,
                (start, end) -> new Trajectory100(
                        List.of(new TimedPose(new Pose2dWithMotion(
                                Pose2d.kZero, new Pose2dWithMotion.MotionDirection(0, 0, 0), 0, 0), 0, 0, 0))),
                controller,
                viz);
        command.initialize();
        assertEquals(0, fixture.drive.getPose().getX(), DELTA);
        command.execute();
        command.end(false);
    }

    /** Demonstrate how to use DriveToWaypoint to go to apriltags. */
    @Test
    void testAprilTag() throws IOException {
        SwerveDriveSubsystem drive = fixture.drive;
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        Transform2d transform = new Transform2d(
                new Translation2d(-1, -1),
                Rotation2d.kZero);

        Pose2d goal = layout.getTagPose(Alliance.Blue, 1).get().toPose2d().plus(transform);
        assertEquals(15.300, goal.getX(), DELTA);
        assertEquals(0.876, goal.getY(), DELTA);
        assertEquals(-0.942, goal.getRotation().getRadians(), DELTA);

        ControllerR3 m_controller = ControllerFactoryR3.test(logger);
        DriveToPoseWithTrajectory command = new DriveToPoseWithTrajectory(
                () -> goal, drive,
                (start, end) -> planner.movingToRest(start, end),
                m_controller, viz);
        command.initialize();
        assertEquals(0, fixture.drive.getPose().getX(), DELTA);
        command.execute();
        command.end(false);
    }

}

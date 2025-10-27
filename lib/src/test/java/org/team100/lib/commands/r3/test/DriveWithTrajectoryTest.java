package org.team100.lib.commands.r3.test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.r3.ControllerFactoryR3;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.swerve.Fixtured;
import org.team100.lib.motion.swerve.SwerveDriveSubsystem;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.MockSubsystemR3;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.TimingConstraintFactory;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveWithTrajectoryTest extends Fixtured implements Timeless {
    public DriveWithTrajectoryTest() throws IOException {
    }

    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final TrajectoryVisualization viz = new TrajectoryVisualization(logger);
    SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
    List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood(logger);
    TrajectoryPlanner planner = new TrajectoryPlanner(constraints);

    @Test
    void testTrajectoryStart() {
        Trajectory100 t = planner.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
        // first state is motionless
        assertEquals(0, t.sample(0).velocityM_S(), DELTA);
        ControllerR3 controller = ControllerFactoryR3.test(logger);

        // initially at rest
        MockSubsystemR3 d = new MockSubsystemR3(new ModelR3());

        DriveWithTrajectory c = new DriveWithTrajectory(d, controller, t, viz);

        stepTime();
        c.initialize();
        c.execute();
        // assertEquals(0.098, d.m_atRestSetpoint.x(), DELTA);
        // assertEquals(0, d.m_atRestSetpoint.y(), DELTA);
        // assertEquals(0, d.m_atRestSetpoint.theta(), DELTA);

        // we don't advance because we're still steering.
        // this next-setpoint is from "preview"
        // and our current setpoint is equal to the measurement.
        stepTime();
        c.execute();
        // assertEquals(0.098, d.m_atRestSetpoint.x(), DELTA);
        // assertEquals(0, d.m_atRestSetpoint.y(), DELTA);
        // assertEquals(0, d.m_atRestSetpoint.theta(), DELTA);

        stepTime();
        c.execute();
        assertEquals(0.102, d.m_setpoint.x(), DELTA);
        assertEquals(0, d.m_setpoint.y(), DELTA);
        assertEquals(0, d.m_setpoint.theta(), DELTA);

        // more normal driving
        stepTime();
        c.execute();
        assertEquals(0.139, d.m_setpoint.x(), DELTA);
        assertEquals(0, d.m_setpoint.y(), DELTA);
        assertEquals(0, d.m_setpoint.theta(), DELTA);

        // etc
        stepTime();
        c.execute();
        assertEquals(0.179, d.m_setpoint.x(), DELTA);
        assertEquals(0, d.m_setpoint.y(), DELTA);
        assertEquals(0, d.m_setpoint.theta(), DELTA);
    }

    @Test
    void testTrajectoryDone() {
        Trajectory100 t = planner.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
        // first state is motionless
        assertEquals(0, t.sample(0).velocityM_S(), DELTA);
        ControllerR3 controller = ControllerFactoryR3.test(logger);

        // initially at rest
        MockSubsystemR3 d = new MockSubsystemR3(new ModelR3());
        
        DriveWithTrajectory c = new DriveWithTrajectory(d, controller, t, viz);
        c.initialize();

        for (int i = 0; i < 100; ++i) {
            stepTime();
            c.execute();
            // we have magically reached the end
            d.m_state = new ModelR3(new Pose2d(1, 0, Rotation2d.kZero));
        }
        assertTrue(c.isDone());

    }

    /** Use a real drivetrain to observe the effect on the motors etc. */
    @Test
    void testRealDrive() {
        fixture.collection.reset();
        // this test depends on the behavior of the setpoint generator, so make sure
        // it's on (otherwise it's in whatever state the previous test left it)
        Experiments.instance.testOverride(Experiment.UseSetpointGenerator, true);
        // 1m along +x, no rotation.
        Trajectory100 trajectory = planner.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
        // first state is motionless
        assertEquals(0, trajectory.sample(0).velocityM_S(), DELTA);
        ControllerR3 controller = ControllerFactoryR3.test(logger);

        SwerveDriveSubsystem drive = fixture.drive;

        // initially at rest
        assertEquals(0, fixture.collection.states().frontLeft().speedMetersPerSecond(), DELTA);
        assertEquals(0, fixture.collection.states().frontLeft().angle().get().getRadians(), DELTA);

        DriveWithTrajectory command = new DriveWithTrajectory(drive, controller, trajectory, viz);
        stepTime();
        command.initialize();

        command.execute();
        // but that output is not available until after takt.
        assertEquals(0, fixture.collection.states().frontLeft().speedMetersPerSecond(), DELTA);
        assertEquals(0, fixture.collection.states().frontLeft().angle().get().getRadians(), DELTA);

        // drive normally more
        stepTime();
        command.execute();
        // this is the output from the previous takt
        assertEquals(0.033, fixture.collection.states().frontLeft().speedMetersPerSecond(), DELTA);
        assertEquals(0, fixture.collection.states().frontLeft().angle().get().getRadians(), DELTA);

        // etc
        stepTime();
        command.execute();
        assertEquals(0.064, fixture.collection.states().frontLeft().speedMetersPerSecond(), DELTA);
        assertEquals(0, fixture.collection.states().frontLeft().angle().get().getRadians(), DELTA);
    }

}

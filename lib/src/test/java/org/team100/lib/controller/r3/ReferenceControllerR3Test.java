package org.team100.lib.controller.r3;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.swerve.Fixtured;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.reference.r3.TrajectoryReferenceR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.MockSubsystemR3;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.path.Path100;
import org.team100.lib.trajectory.path.PathFactory;
import org.team100.lib.trajectory.timing.ScheduleGenerator;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.TimingConstraintFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ReferenceControllerR3Test extends Fixtured implements Timeless {
    public ReferenceControllerR3Test() throws IOException {

    }

    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
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
        MockSubsystemR3 drive = new MockSubsystemR3(new ModelR3());

        TrajectoryReferenceR3 reference = new TrajectoryReferenceR3(t);
        ReferenceControllerR3 c = new ReferenceControllerR3(
                drive, controller, reference);

        stepTime();
        c.execute();

        // we don't advance because we're still steering.
        // this next-setpoint is from "preview"
        // and our current setpoint is equal to the measurement.
        stepTime();
        c.execute();
        // assertEquals(0.098, drive.m_atRestSetpoint.x(), DELTA);
        // assertEquals(0, drive.m_atRestSetpoint.y(), DELTA);
        // assertEquals(0, drive.m_atRestSetpoint.theta(), DELTA);

        stepTime();
        c.execute();
        assertEquals(0.139, drive.m_setpoint.x(), DELTA);
        assertEquals(0, drive.m_setpoint.y(), DELTA);
        assertEquals(0, drive.m_setpoint.theta(), DELTA);

        // more normal driving
        stepTime();
        c.execute();
        assertEquals(0.179, drive.m_setpoint.x(), DELTA);
        assertEquals(0, drive.m_setpoint.y(), DELTA);
        assertEquals(0, drive.m_setpoint.theta(), DELTA);

        // etc
        stepTime();
        c.execute();
        assertEquals(0.221, drive.m_setpoint.x(), DELTA);
        assertEquals(0, drive.m_setpoint.y(), DELTA);
        assertEquals(0, drive.m_setpoint.theta(), DELTA);
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
        MockSubsystemR3 drive = new MockSubsystemR3(new ModelR3());

        TrajectoryReferenceR3 reference = new TrajectoryReferenceR3(t);
        ReferenceControllerR3 c = new ReferenceControllerR3(
                drive, controller, reference);

        // the measurement never changes but that doesn't affect "done" as far as the
        // trajectory is concerned.
        for (int i = 0; i < 100; ++i) {
            stepTime();
            c.execute();
            // we have magically reached the end
            drive.m_state = new ModelR3(new Pose2d(1, 0, Rotation2d.kZero));
        }
        assertTrue(c.isDone());

    }

    @Test
    void testFieldRelativeTrajectory() {
        List<HolonomicPose2d> waypoints = new ArrayList<>();
        waypoints.add(new HolonomicPose2d(
                new Translation2d(), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)));
        waypoints.add(new HolonomicPose2d(
                new Translation2d(100, 4), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)));
        waypoints.add(new HolonomicPose2d(
                new Translation2d(196, 13), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));

        double start_vel = 0.0;
        double end_vel = 0.0;

        Path100 path = PathFactory.pathFromWaypoints(waypoints, 2, 0.25, 0.1);
        assertFalse(path.isEmpty());

        double stepSize = 2;
        ScheduleGenerator u = new ScheduleGenerator(Arrays.asList());
        Trajectory100 trajectory = u.timeParameterizeTrajectory(
                path,
                stepSize,
                start_vel,
                end_vel);
        if (DEBUG)
            System.out.printf("TRAJECTORY:\n%s\n", trajectory);

        FullStateControllerR3 swerveController = new FullStateControllerR3(
                logger,
                2.4, 2.4,
                0.1, 0.1,
                0.01, 0.02,
                0.01, 0.02);

        MockSubsystemR3 drive = new MockSubsystemR3(new ModelR3());
        TrajectoryReferenceR3 reference = new TrajectoryReferenceR3(trajectory);
        ReferenceControllerR3 referenceController = new ReferenceControllerR3(
                drive, swerveController, reference);

        Pose2d pose = trajectory.sample(0).state().getPose();
        GlobalVelocityR3 velocity = GlobalVelocityR3.zero();

        double mDt = 0.02;
        int i = 0;
        while (!referenceController.isDone()) {
            if (++i > 500)
                break;
            stepTime();
            drive.m_state = new ModelR3(pose, velocity);
            referenceController.execute();
            velocity = drive.m_recentSetpoint;
            pose = new Pose2d(
                    pose.getX() + velocity.x() * mDt,
                    pose.getY() + velocity.y() * mDt,
                    new Rotation2d(pose.getRotation().getRadians() + velocity.theta() * mDt));
            if (DEBUG)
                System.out.printf("pose %s vel %s\n", pose, velocity);
        }

        // this should be exactly right but it's not.
        assertEquals(195, pose.getTranslation().getX(), 1);
        assertEquals(13, pose.getTranslation().getY(), 0.4);
        assertEquals(0, pose.getRotation().getRadians(), 0.1);
    }

}

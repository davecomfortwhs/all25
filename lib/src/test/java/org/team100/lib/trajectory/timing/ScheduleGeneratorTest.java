package org.team100.lib.trajectory.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.Pose2dWithMotion.MotionDirection;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.path.Path100;
import org.team100.lib.trajectory.path.PathFactory;
import org.team100.lib.trajectory.timing.TimingConstraint.MinMaxAcceleration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ScheduleGeneratorTest {
    private static final boolean DEBUG = false;
    public static final double EPSILON = 1e-12;
    private static final double DELTA = 0.01;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    public static final List<Pose2dWithMotion> WAYPOINTS = Arrays.asList(
            new Pose2dWithMotion(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.kZero)),
            new Pose2dWithMotion(new Pose2d(new Translation2d(24.0, 0.0), Rotation2d.kZero)),
            new Pose2dWithMotion(new Pose2d(new Translation2d(36.0, 12.0), Rotation2d.kZero)),
            new Pose2dWithMotion(new Pose2d(new Translation2d(60.0, 12.0), Rotation2d.kZero)));

    public static final List<Rotation2d> HEADINGS = List.of(
            GeometryUtil.fromDegrees(0),
            GeometryUtil.fromDegrees(0),
            GeometryUtil.fromDegrees(0),
            GeometryUtil.fromDegrees(0));

    public Trajectory100 buildAndCheckTrajectory(
            final Path100 path,
            double step_size,
            List<TimingConstraint> constraints,
            double start_vel,
            double end_vel,
            double max_vel,
            double max_acc) {
        ScheduleGenerator u = new ScheduleGenerator(constraints);
        Trajectory100 timed_traj = u.timeParameterizeTrajectory(
                path,
                step_size,
                start_vel,
                end_vel);
        checkTrajectory(timed_traj, constraints, start_vel, end_vel, max_vel, max_acc);
        return timed_traj;
    }

    public void checkTrajectory(
            final Trajectory100 traj,
            List<TimingConstraint> constraints,
            double start_vel,
            double end_vel,
            double max_vel,
            double max_acc) {
        assertFalse(traj.isEmpty());
        assertEquals(start_vel, traj.sample(0).velocityM_S(), EPSILON);
        assertEquals(end_vel, traj.getLastPoint().velocityM_S(), EPSILON);

        // Go state by state, verifying all constraints are satisfied and integration is
        // correct.
        TimedPose prev_state = null;
        for (TimedPose state : traj.getPoints()) {
            for (final TimingConstraint constraint : constraints) {
                assertTrue(state.velocityM_S() - EPSILON <= constraint.getMaxVelocity(state.state()).getValue());
                final MinMaxAcceleration accel_limits = constraint.getMinMaxAcceleration(state.state(),
                        state.velocityM_S());
                assertTrue(state.acceleration() - EPSILON <= accel_limits.getMaxAccel(),
                        String.format("%f %f", state.acceleration(), accel_limits.getMaxAccel()));
                assertTrue(state.acceleration() + EPSILON >= accel_limits.getMinAccel(),
                        String.format("%f %f", state.acceleration(), accel_limits.getMinAccel()));
            }
            if (prev_state != null) {
                assertEquals(state.velocityM_S(),
                        prev_state.velocityM_S()
                                + (state.getTimeS() - prev_state.getTimeS()) * prev_state.acceleration(),
                        EPSILON);
            }
            prev_state = state;
        }
    }

    /**
     * Turning in place does not work.
     */
    @Test
    void testJustTurningInPlace() {
        Path100 path = new Path100(Arrays.asList(
                new Pose2dWithMotion(
                        new Pose2d(
                                new Translation2d(0.0, 0.0),
                                Rotation2d.kZero),
                        new MotionDirection(0, 0, 1), 0, 0),
                new Pose2dWithMotion(
                        new Pose2d(
                                new Translation2d(0.0, 0.0),
                                Rotation2d.kPi),
                        new MotionDirection(0, 0, 1), 0, 0)));

        // Triangle profile.
        assertThrows(IllegalArgumentException.class,
                () -> buildAndCheckTrajectory(
                        path, 1.0, new ArrayList<TimingConstraint>(), 0.0, 0.0, 20.0, 5.0));

        // Trapezoidal profile.
        assertThrows(IllegalArgumentException.class,
                () -> buildAndCheckTrajectory(
                        path, 1.0, new ArrayList<TimingConstraint>(), 0.0, 0.0, 10.0, 5.0));

        // Trapezoidal profile with start and end velocities.
        assertThrows(IllegalArgumentException.class,
                () -> buildAndCheckTrajectory(
                        path, 1.0, new ArrayList<TimingConstraint>(), 5.0, 2.0, 10.0, 5.0));
    }

    /**
     * The path here is just four waypoints, so sharp corners.
     * 
     * The trajectory just notices velocity and acceleration along the path, so it
     * is totally infeasible at the corners.
     */
    @Test
    void testNoConstraints() {
        Path100 path = new Path100(WAYPOINTS);

        // Triangle profile.
        Trajectory100 timed_traj = buildAndCheckTrajectory(path,
                1.0,
                new ArrayList<TimingConstraint>(), 0.0, 0.0, 20.0, 5.0);
        assertEquals(66, timed_traj.length());

        // Trapezoidal profile.
        timed_traj = buildAndCheckTrajectory(path,
                1.0, new ArrayList<TimingConstraint>(),
                0.0, 0.0,
                10.0, 5.0);
        assertEquals(66, timed_traj.length());

        // Trapezoidal profile with start and end velocities.
        timed_traj = buildAndCheckTrajectory(path,
                1.0, new ArrayList<TimingConstraint>(),
                5.0, 2.0,
                10.0, 5.0);
        assertEquals(66, timed_traj.length());
    }

    /**
     * The centripetal constraint does nothing in the corners, because these paths
     * aren't realistic; the corners are ignored here.
     */
    @Test
    void testCentripetalConstraint() {
        Path100 path = new Path100(WAYPOINTS);
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forRealisticTest();

        // Triangle profile.
        Trajectory100 timed_traj = buildAndCheckTrajectory(path,
                1.0,
                List.of(new CapsizeAccelerationConstraint(logger, limits, 1.0)), 0.0, 0.0, 20.0, 5.0);
        assertEquals(66, timed_traj.length());
        assertNotNull(timed_traj);

        // Trapezoidal profile.
        timed_traj = buildAndCheckTrajectory(path, 1.0, new ArrayList<TimingConstraint>(), 0.0, 0.0,
                10.0, 5.0);
        assertEquals(66, timed_traj.length());

        // Trapezoidal profile with start and end velocities.
        timed_traj = buildAndCheckTrajectory(path, 1.0, new ArrayList<TimingConstraint>(), 5.0, 2.0,
                10.0, 5.0);
        assertEquals(66, timed_traj.length());
    }

    @Test
    void testConditionalVelocityConstraint() {
        Path100 path = new Path100(WAYPOINTS);

        class ConditionalTimingConstraint implements TimingConstraint {
            @Override
            public NonNegativeDouble getMaxVelocity(Pose2dWithMotion state) {
                if (state.getTranslation().getX() >= 24.0) {
                    return new NonNegativeDouble(5.0);
                } else {
                    return new NonNegativeDouble(Double.POSITIVE_INFINITY);
                }
            }

            @Override
            public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state,
                    double velocity) {
                return new TimingConstraint.MinMaxAcceleration(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
            }
        }

        // Trapezoidal profile.
        Trajectory100 timed_traj = buildAndCheckTrajectory(
                path, 1.0, Arrays.asList(new ConditionalTimingConstraint()), 0.0, 0.0, 10.0, 5.0);
        assertNotNull(timed_traj);
    }

    @Test
    void testConditionalAccelerationConstraint() {
        Path100 path = new Path100(WAYPOINTS);

        class ConditionalTimingConstraint implements TimingConstraint {
            @Override
            public NonNegativeDouble getMaxVelocity(Pose2dWithMotion state) {
                return new NonNegativeDouble(Double.POSITIVE_INFINITY);
            }

            @Override
            public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state,
                    double velocity) {
                return new TimingConstraint.MinMaxAcceleration(-10.0, 10.0 / velocity);
            }
        }

        // Trapezoidal profile.
        Trajectory100 timed_traj = buildAndCheckTrajectory(
                path, 1.0, Arrays.asList(new ConditionalTimingConstraint()), 0.0, 0.0, 10.0, 5.0);
        assertNotNull(timed_traj);
    }

    @Test
    void testAccel() {
        // average v = 0.5
        // dv = 1
        assertEquals(0.5, ScheduleGenerator.accel(0, 1, 1.0), 0.001);
        assertEquals(1.0, ScheduleGenerator.accel(0, 1, 0.5), 0.001);
        // average v = 1.5
        // dv = 1
        assertEquals(1.5, ScheduleGenerator.accel(1, 2, 1.0), 0.001);
        // same case, backwards
        assertEquals(1.5, ScheduleGenerator.accel(2, 1, -1.0), 0.001);
    }

    @Test
    void testV1() {
        // no v or a => no new v
        assertEquals(0.0, ScheduleGenerator.v1(0, 0, 1.0));
        // no a => keep old v
        assertEquals(1.0, ScheduleGenerator.v1(1, 0, 1.0));
        // a = 0.5 for 1 => final v is 1
        assertEquals(1.0, ScheduleGenerator.v1(0, 0.5, 1.0));
        // same case, backwards
        assertEquals(0.0, ScheduleGenerator.v1(1.0, 0.5, -1.0));
        // backwards with negative accel
        assertEquals(1.0, ScheduleGenerator.v1(0.0, -0.5, -1.0));
    }

    /**
     * 0.16 ms on my machine.
     * 
     * See PathFactoryTest::testPerformance()
     */
    @Test
    void testPerformance() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 1), new Rotation2d(), new Rotation2d(Math.PI / 2)));
        long startTimeNs = System.nanoTime();
        final int iterations = 100;
        final double SPLINE_SAMPLE_TOLERANCE_M = 0.05;
        final double SPLINE_SAMPLE_TOLERANCE_RAD = 0.2;
        final double TRAJECTORY_STEP_M = 0.1;

        Path100 path = PathFactory.pathFromWaypoints(
                waypoints,
                SPLINE_SAMPLE_TOLERANCE_M,
                SPLINE_SAMPLE_TOLERANCE_M,
                SPLINE_SAMPLE_TOLERANCE_RAD);
        Trajectory100 t = new Trajectory100();
        ScheduleGenerator m_scheduleGenerator = new ScheduleGenerator(new ArrayList<>());
        for (int i = 0; i < iterations; ++i) {
            t = m_scheduleGenerator.timeParameterizeTrajectory(
                    path,
                    TRAJECTORY_STEP_M,
                    0,
                    0);
        }
        long endTimeNs = System.nanoTime();
        double totalDurationMs = (endTimeNs - startTimeNs) / 1000000.0;
        if (DEBUG) {
            System.out.printf("total duration ms: %5.3f\n", totalDurationMs);
            System.out.printf("duration per iteration ms: %5.3f\n", totalDurationMs / iterations);
        }
        assertEquals(18, t.length());
        TimedPose p = t.getPoint(6);
        assertEquals(0.575, p.state().getPose().getX(), DELTA);
        assertEquals(0, p.state().getHeadingRate(), DELTA);

    }

}

package org.team100.lib.trajectory.path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.path.spline.HolonomicSpline;
import org.team100.lib.trajectory.timing.ScheduleGenerator;
import org.team100.lib.trajectory.timing.ScheduleGenerator.TimingException;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.TimingConstraintFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class PathFactoryTest {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.01;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testForced() throws TimingException {
        List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(1, 0, new Rotation2d()),
                new Pose2d(1, 1, new Rotation2d()));
        Path100 path = PathFactory.withoutControlPoints(waypoints, 0.01, 0.01, 0.1);
        // sample the path so we can see it
        for (double t = 0; t <= path.getMaxDistance(); t += 0.1) {
            if (DEBUG)
                System.out.printf("%.1f %5.2f %5.2f\n",
                        t,
                        path.sample(t).getPose().getX(),
                        path.sample(t).getPose().getY());
        }
        // schedule it so we can see it
        // no constraints
        TimingConstraintFactory f = new TimingConstraintFactory(SwerveKinodynamicsFactory.forTest());
        List<TimingConstraint> constraints = f.forTest(logger);
        ScheduleGenerator scheduler = new ScheduleGenerator(constraints);
        Trajectory100 trajectory = scheduler.timeParameterizeTrajectory(
                path,
                0.0127,
                0.05,
                0.05);

        for (double t = 0; t < trajectory.duration(); t += 0.1) {
            if (DEBUG)
                System.out.printf("%.1f %5.2f %5.2f\n",
                        t,
                        trajectory.sample(t).state().getPose().getX(),
                        trajectory.sample(t).state().getPose().getY());
        }

        assertEquals(13, path.length());
        Pose2dWithMotion p = path.getPoint(0);
        assertEquals(0, p.getPose().getX(), DELTA);
        assertEquals(0, p.getPose().getRotation().getRadians(), DELTA);
        assertEquals(0, p.getHeadingRate(), DELTA);
        p = path.getPoint(1);
        assertEquals(0.3, p.getPose().getX(), DELTA);
        assertEquals(0, p.getPose().getRotation().getRadians(), DELTA);
        assertEquals(0, p.getHeadingRate(), DELTA);
    }

    @Test
    void testBackingUp() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(0, 0), Rotation2d.kZero, new Rotation2d(Math.PI)),
                new HolonomicPose2d(new Translation2d(1, 0), Rotation2d.kZero, Rotation2d.kZero));
        Path100 path = PathFactory.pathFromWaypoints(
                waypoints,
                0.0127,
                0.0127,
                Math.toRadians(1.0));
        assertEquals(10, path.length());
    }

    /**
     * Stationary pure-rotation paths don't work.
     */
    @Test
    void testRotation() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(), new Rotation2d(1), new Rotation2d()));
        Path100 path = PathFactory.pathFromWaypoints(waypoints, 0.01, 0.01, 0.1);
        assertEquals(1, path.length());
    }

    /** Preserves the tangent at the corner and so makes a little "S" */
    @Test
    void testCorner() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(0, 0), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 0), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 1), new Rotation2d(), new Rotation2d(Math.PI / 2)));
        Path100 path = PathFactory.pathFromWaypoints(waypoints, 0.01, 0.01, 0.1);

        assertEquals(9, path.length());
        Pose2dWithMotion p = path.getPoint(0);
        assertEquals(0, p.getPose().getX(), DELTA);
        assertEquals(0, p.getPose().getRotation().getRadians(), DELTA);
        assertEquals(0, p.getHeadingRate(), DELTA);
        p = path.getPoint(1);
        assertEquals(1, p.getPose().getX(), DELTA);
        assertEquals(0, p.getPose().getRotation().getRadians(), DELTA);
        assertEquals(0, p.getHeadingRate(), DELTA);
    }

    /**
     * Stationary paths don't work.
     */
    @Test
    void testStationary() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(), new Rotation2d(), new Rotation2d()));
        Path100 path = PathFactory.pathFromWaypoints(waypoints, 0.01, 0.01, 0.1);
        assertEquals(1, path.length());
    }

    @Test
    void testLinear() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 0), new Rotation2d(), new Rotation2d()));
        Path100 path = PathFactory.pathFromWaypoints(
                waypoints, 0.01, 0.01, 0.1);
        assertEquals(2, path.length());
        Pose2dWithMotion p = path.getPoint(0);
        assertEquals(0, p.getPose().getX(), DELTA);
        assertEquals(0, p.getPose().getRotation().getRadians(), DELTA);
        assertEquals(0, p.getHeadingRate(), DELTA);
        p = path.getPoint(1);
        assertEquals(1, p.getPose().getX(), DELTA);
        assertEquals(0, p.getPose().getRotation().getRadians(), DELTA);
        assertEquals(0, p.getHeadingRate(), DELTA);
    }

    @Test
    void testActualCorner() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(0, 0), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 0), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 0), new Rotation2d(), new Rotation2d(Math.PI / 2)),
                new HolonomicPose2d(new Translation2d(1, 1), new Rotation2d(), new Rotation2d(Math.PI / 2)));
        assertThrows(IllegalArgumentException.class,
                () -> PathFactory.pathFromWaypoints(waypoints, 0.01, 0.01, 0.1));
    }

    @Test
    void testComposite() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(0, 0), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 0), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 0), new Rotation2d(1), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(2, 0), new Rotation2d(1), new Rotation2d()));
        assertThrows(IllegalArgumentException.class,
                () -> PathFactory.pathFromWaypoints(waypoints, 0.01, 0.01, 0.1));
    }

    @Test
    void test() {
        HolonomicPose2d p1 = new HolonomicPose2d(new Translation2d(0, 0), Rotation2d.kZero, Rotation2d.kZero);
        HolonomicPose2d p2 = new HolonomicPose2d(new Translation2d(15, 10), Rotation2d.kZero, new Rotation2d(1, 5));
        HolonomicSpline s = new HolonomicSpline(p1, p2);

        List<Pose2dWithMotion> samples = PathFactory.parameterizeSpline(s, 0.05, 0.05, 0.1, 0.0, 1.0);

        double arclength = 0;
        Pose2dWithMotion cur_pose = samples.get(0);
        for (Pose2dWithMotion sample : samples) {
            Twist2d twist = GeometryUtil.slog(
                    GeometryUtil.transformBy(
                            GeometryUtil.inverse(cur_pose.getPose()), sample.getPose()));
            arclength += Math.hypot(twist.dx, twist.dy);
            cur_pose = sample;
        }

        assertEquals(15.0, cur_pose.getTranslation().getX(), 0.001);
        assertEquals(10.0, cur_pose.getTranslation().getY(), 0.001);
        assertEquals(78.690, cur_pose.getCourse().get().getDegrees(), 0.001);
        assertEquals(20.416, arclength, 0.001);
    }

    @Test
    void testDx() {
        HolonomicSpline s0 = new HolonomicSpline(
                new HolonomicPose2d(new Translation2d(0, -1), Rotation2d.kZero, Rotation2d.kZero),
                new HolonomicPose2d(new Translation2d(1, 0), Rotation2d.kZero, Rotation2d.kCCW_90deg),
                1.0, 1.0);
        List<HolonomicSpline> splines = List.of(s0);
        List<Pose2dWithMotion> motion = PathFactory.parameterizeSplines(splines, 0.001, 0.001, 0.001);
        for (Pose2dWithMotion p : motion) {
            if (DEBUG)
                System.out.printf("%5.3f %5.3f\n", p.getTranslation().getX(), p.getTranslation().getY());
        }
    }

    /**
     * 0.15 ms on my machine.
     * 
     * See TrajectoryPlannerTest::testPerformance().
     */
    @Test
    void testPerformance() {
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(), new Rotation2d(), new Rotation2d()),
                new HolonomicPose2d(new Translation2d(1, 1), new Rotation2d(), new Rotation2d(Math.PI / 2)));
        long startTimeNs = System.nanoTime();
        Path100 t = new Path100(new ArrayList<>());
        final int iterations = 100;
        final double SPLINE_SAMPLE_TOLERANCE_M = 0.05;
        final double SPLINE_SAMPLE_TOLERANCE_RAD = 0.2;
        for (int i = 0; i < iterations; ++i) {
            t = PathFactory.pathFromWaypoints(
                    waypoints,
                    SPLINE_SAMPLE_TOLERANCE_M,
                    SPLINE_SAMPLE_TOLERANCE_M,
                    SPLINE_SAMPLE_TOLERANCE_RAD);
        }
        long endTimeNs = System.nanoTime();
        double totalDurationMs = (endTimeNs - startTimeNs) / 1000000.0;
        if (DEBUG) {
            System.out.printf("total duration ms: %5.3f\n", totalDurationMs);
            System.out.printf("duration per iteration ms: %5.3f\n", totalDurationMs / iterations);
        }
        assertEquals(5, t.length());
        Pose2dWithMotion p = t.getPoint(1);
        assertEquals(0.417, p.getPose().getX(), DELTA);
        assertEquals(0, p.getHeadingRate(), DELTA);
    }

}

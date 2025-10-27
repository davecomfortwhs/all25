package org.team100.lib.trajectory.timing;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.path.Path100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Verify that trajectory schedule generation yields a realistic profile.
 * 
 * see
 * https://docs.google.com/spreadsheets/d/16UUCCz-qcPz_YZMnsJnVkVO1KGp5zHCOVo7EoJct2nA/edit?gid=0#gid=0
 */
public class TrajectoryVelocityProfileTest {
    private static final boolean DEBUG = false;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    // A five-meter straight line.
    public static final List<Pose2dWithMotion> WAYPOINTS = Arrays.asList(
            new Pose2dWithMotion(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.kZero)),
            new Pose2dWithMotion(new Pose2d(new Translation2d(5.0, 0.0), Rotation2d.kZero)));

    // No rotation.
    public static final List<Rotation2d> HEADINGS = List.of(
            GeometryUtil.fromDegrees(0),
            GeometryUtil.fromDegrees(0));

    void print(Trajectory100 traj) {
        if (!DEBUG)
            return;
        for (double t = 0; t < traj.duration(); t += 0.02) {
            System.out.printf("%12.6f %12.6f\n", t, traj.sample(t).velocityM_S());
        }

    }

    /** This uses the default max accel which is ridiculously high. */
    @Test
    void testNoConstraint() {
        Path100 path = new Path100(WAYPOINTS);
        List<TimingConstraint> constraints = new ArrayList<TimingConstraint>();
        ScheduleGenerator u = new ScheduleGenerator(constraints);
        Trajectory100 traj = u.timeParameterizeTrajectory(
                path, 0.1, 0, 0);
        print(traj);
    }

    /** This produces a trapezoid. */
    @Test
    void testConstantConstraint() {
        Path100 path = new Path100(WAYPOINTS);
        // somewhat realistic numbers
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTrajectoryTimingTest();
        List<TimingConstraint> constraints = List.of(new ConstantConstraint(logger, 1, 1, limits));
        ScheduleGenerator u = new ScheduleGenerator(constraints);
        Trajectory100 traj = u.timeParameterizeTrajectory(
                path, 0.1, 0, 0);
        print(traj);
    }

    /** This produces the desired current-limited exponential shape. */
    @Test
    void testSwerveConstraint() {
        Path100 path = new Path100(WAYPOINTS);
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTrajectoryTimingTest();
        List<TimingConstraint> constraints = List.of(new SwerveDriveDynamicsConstraint(logger, limits, 1, 1));
        ScheduleGenerator u = new ScheduleGenerator(constraints);
        Trajectory100 traj = u.timeParameterizeTrajectory(
                path, 0.1, 0, 0);
        print(traj);
    }

    /** */
    @Test
    void testAuto() {
        Path100 path = new Path100(WAYPOINTS);
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTrajectoryTimingTest();
        TimingConstraintFactory timing = new TimingConstraintFactory(limits);
        List<TimingConstraint> constraints = timing.testAuto(logger);
        ScheduleGenerator u = new ScheduleGenerator(constraints);
        Trajectory100 traj = u.timeParameterizeTrajectory(
                path, 0.1, 0, 0);
        print(traj);
    }
}

package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;

import org.team100.lib.gyro.Gyro;
import org.team100.lib.gyro.MockGyro;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.motion.swerve.module.state.SwerveModulePositions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveDrivePoseEstimator100PerformanceTest {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;

    private final Pose2d visionRobotPoseMeters = new Pose2d(1, 0, Rotation2d.kZero);

    static SwerveModulePositions p(double x) {
        SwerveModulePosition100 m = new SwerveModulePosition100(x, Optional.of(Rotation2d.kZero));
        return new SwerveModulePositions(m, m, m, m);
    }

    private SwerveModulePositions positions;

    /*
     * Should we optimize the pose replay operation?
     * 
     * On my machine, an i5-9400f, 2.9ghz, each update takes about 3 microseconds.
     * 
     * The RoboRIO is about 3.5x slower than my machine, so say 10 microseconds per
     * update.
     * 
     * Each camera may provide zero or one updates per roboRIO cycle, so the worst
     * case would be 50 microseconds for these pose buffer updates.
     * 
     * The total budget is 20000 microseconds, but it would be good to be way under
     * that, so say 5000 microseconds. So pose buffer updates may consume 1% of
     * the budget in the worst case.
     * 
     * The average case is about 3 frames per cycle, so 30 microseconds or 0.6%,
     * which is significant but maybe not urgent?
     * 
     * If we *did* want to optimize it, we could move the replay from the vision
     * writer to the pose reader, and save something like 20 us (0.4%) on average.
     */
    // There's no need to run this all the time
    // @Test
    void test0() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        double[] stateStdDevs = new double[] { 0.1, 0.1, 0.1 };
        double[] visionMeasurementStdDevs = new double[] { 0.5, 0.5, Double.MAX_VALUE };

        Gyro gyro = new MockGyro();
        SwerveHistory history = new SwerveHistory(
                kinodynamics,
                Rotation2d.kZero,
                SwerveModulePositions.kZero(),
                Pose2d.kZero,
                0);
        positions = p(0);
        OdometryUpdater ou = new OdometryUpdater(kinodynamics, gyro, history, () -> positions);
        ou.reset(Pose2d.kZero, 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(history, ou);

        // fill the buffer with odometry
        double t = 0.0;
        double duration = 0.2; // SwerveDrivePoseEstimator100.BUFFER_DURATION;
        while (t < duration) {
            positions = p(t);
            ou.update(t);
            t += 0.02;
        }
        assertEquals(11, history.size());
        assertEquals(0.2, history.lastKey(), DELTA);

        // add a very old vision estimate, which triggers replay of the entire buffer.
        int iterations = 100000;
        long startTime = System.currentTimeMillis();
        for (int i = 0; i < iterations; ++i) {
            vu.put(0.00, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        }
        long finishTime = System.currentTimeMillis();
        if (DEBUG) {
            System.out.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
            System.out.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
        }
        assertEquals(11, history.size());
        assertEquals(0.2, history.lastKey(), DELTA);

        // add a recent vision estimate, which triggers replay of a few samples.
        iterations = 1000000;
        startTime = System.currentTimeMillis();
        for (int i = 0; i < iterations; ++i) {
            vu.put(duration - 0.1, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        }
        finishTime = System.currentTimeMillis();
        if (DEBUG) {
            System.out.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
            System.out.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
        }
        assertEquals(11, history.size());
        assertEquals(0.2, history.lastKey(), DELTA);
    }
}

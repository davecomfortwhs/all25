package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.TreeMap;
import java.util.function.Function;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.gyro.Gyro;
import org.team100.lib.gyro.MockGyro;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.motion.swerve.module.state.SwerveModulePositions;
import org.team100.lib.motion.swerve.module.state.SwerveModuleState100;
import org.team100.lib.motion.swerve.module.state.SwerveModuleStates;
import org.team100.lib.state.ModelR3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DataLogManager;

class SwerveDrivePoseEstimator100Test {
    private static final double DELTA = 0.001;
    private static final boolean DEBUG = false;

    private final SwerveModulePosition100 p0 = new SwerveModulePosition100(0, Optional.of(Rotation2d.kZero));
    private final SwerveModulePositions positionZero = new SwerveModulePositions(p0, p0, p0, p0);
    private final SwerveModulePosition100 p01 = new SwerveModulePosition100(0.1,
            Optional.of(Rotation2d.kZero));
    private final SwerveModulePositions position01 = new SwerveModulePositions(p01, p01, p01, p01);
    private final Pose2d visionRobotPoseMeters = new Pose2d(1, 0, Rotation2d.kZero);

    private SwerveModulePositions positions;

    private static void verify(double x, ModelR3 state) {
        Pose2d estimate = state.pose();
        assertEquals(x, estimate.getX(), DELTA);
        assertEquals(0, estimate.getY(), DELTA);
        assertEquals(0, estimate.getRotation().getRadians(), DELTA);
    }

    private static void verifyVelocity(double xV, ModelR3 state) {
        GlobalVelocityR3 v = state.velocity();
        assertEquals(xV, v.x(), DELTA);
    }

    @BeforeEach
    void nolog() {
        DataLogManager.stop();
    }

    @Test
    void testGyroOffset() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();
        SwerveHistory history = new SwerveHistory(
                kinodynamics,
                Rotation2d.kZero,
                positionZero,
                Pose2d.kZero,
                0); // zero initial time

        OdometryUpdater ou = new OdometryUpdater(kinodynamics, gyro, history, () -> positions);
        positions = positionZero;
        ou.reset(Pose2d.kZero, 0);
        Pose2d p = history.apply(0).pose();
        assertEquals(0, p.getX(), DELTA);
        assertEquals(0, p.getY(), DELTA);
        assertEquals(0, p.getRotation().getRadians(), DELTA);
        assertEquals(0, ou.getGyroOffset().getRadians(), DELTA);
        // force the pose to rotate 90
        ou.reset(new Pose2d(0, 0, Rotation2d.kCCW_90deg), 0);
        p = history.apply(0).pose();
        assertEquals(0, p.getX(), DELTA);
        assertEquals(0, p.getY(), DELTA);
        // and we get that back
        assertEquals(Math.PI / 2, p.getRotation().getRadians(), DELTA);
        // and the offset is correct since the gyro itself didn't change.
        assertEquals(Math.PI / 2, ou.getGyroOffset().getRadians(), DELTA);
    }

    @Test
    void odo1() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();

        SwerveHistory history = new SwerveHistory(
                kinodynamics,
                Rotation2d.kZero,
                positionZero,
                Pose2d.kZero,
                0);

        OdometryUpdater ou = new OdometryUpdater(kinodynamics, gyro, history, () -> positions);
        positions = positionZero;
        ou.reset(Pose2d.kZero, 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(history, ou);
        positions = positionZero;
        ou.update(0.0);
        verify(0.000, history.apply(0.00));
        verifyVelocity(0.000, history.apply(0.00));
        // 0.1 m
        positions = position01;
        ou.update(0.02);
        verify(0.000, history.apply(0.00));
        verifyVelocity(0.000, history.apply(0.00));
        // velocity is the delta
        verify(0.1, history.apply(0.02));
        verifyVelocity(5.000, history.apply(0.02));

        // big vision update
        double[] stateStdDevs = new double[] { 0.1, 0.1, 0.1 };
        double[] visionMeasurementStdDevs = new double[] { 0.5, 0.5, Double.MAX_VALUE };
        vu.put(0.00, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        // position slides over there
        verify(0.167, history.apply(0.00));
        verifyVelocity(0.000, history.apply(0.00));
        // odometry adds to that
        verify(0.267, history.apply(0.02));
        // velocity is unchanged.
        verifyVelocity(5.000, history.apply(0.02));
    }

    @Test
    void odo2() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();

        double[] stateStdDevs = new double[] { 0.1, 0.1, 0.1 };
        double[] visionMeasurementStdDevs = new double[] { 0.5, 0.5, Double.MAX_VALUE };
        SwerveHistory history = new SwerveHistory(
                kinodynamics,
                Rotation2d.kZero,
                positionZero,
                Pose2d.kZero,
                0);

        OdometryUpdater ou = new OdometryUpdater(kinodynamics, gyro, history, () -> positions);
        positions = positionZero;
        ou.reset(Pose2d.kZero, 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(history, ou);
        positions = positionZero;
        ou.update(0.0);
        verify(0.000, history.apply(0.00));
        verifyVelocity(0.000, history.apply(0.00));
        // 0.1 m
        positions = position01;
        ou.update(0.02);
        verify(0.000, history.apply(0.00));
        verifyVelocity(0.000, history.apply(0.00));
        // velocity is the delta
        verify(0.1, history.apply(0.02));
        verifyVelocity(5.000, history.apply(0.02));

        // big vision update, but later
        vu.put(0.02, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        // initial position is unchanged
        verify(0.000, history.apply(0.00));
        verifyVelocity(0.000, history.apply(0.00));
        // not sure what's happening here
        verify(0.25, history.apply(0.02));
        // velocity is STILL unchanged, i.e. not consistent with the pose history, which
        // is probably better
        // than making velocity reflect the camera noise.
        verifyVelocity(5.000, history.apply(0.02));
    }

    @Test
    void odo3() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();

        double[] stateStdDevs = new double[] { 0.1, 0.1, 0.1 };
        double[] visionMeasurementStdDevs = new double[] { 0.5, 0.5, Double.MAX_VALUE };
        SwerveHistory history = new SwerveHistory(
                kinodynamics,
                Rotation2d.kZero,
                positionZero,
                Pose2d.kZero,
                0);

        OdometryUpdater ou = new OdometryUpdater(kinodynamics, gyro, history, () -> positions);
        positions = positionZero;
        ou.reset(Pose2d.kZero, 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(history, ou);
        positions = positionZero;
        ou.update(0.0);
        verify(0.000, history.apply(0.00));
        verifyVelocity(0.000, history.apply(0.00));
        // 0.1 m
        positions = position01;
        ou.update(0.02);
        verify(0.000, history.apply(0.00));
        verifyVelocity(0.000, history.apply(0.00));
        // velocity is the delta
        verify(0.1, history.apply(0.02));
        verifyVelocity(5.000, history.apply(0.02));

        // big vision update, even later
        vu.put(0.04, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        // initial position is unchanged
        verify(0.000, history.apply(0.00));
        verifyVelocity(0.000, history.apply(0.00));
        // camera does nothing
        verify(0.1, history.apply(0.02));
        // velocity is STILL unchanged, i.e. not consistent with the post history, which
        // is probably better
        // than making velocity reflect the camera noise.
        verifyVelocity(5.000, history.apply(0.02));
        // this is 0.1 towards the camera 1.0
        verify(0.25, history.apply(0.04));
        // still velo is unaffected
        verifyVelocity(5.000, history.apply(0.04));
    }

    @Test
    void outOfOrder() {
        // out of order vision updates
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();

        double[] stateStdDevs = new double[] { 0.1, 0.1, 0.1 };
        double[] visionMeasurementStdDevs = new double[] { 0.5, 0.5, Double.MAX_VALUE };
        positions = positionZero;
        SwerveHistory history = new SwerveHistory(
                kinodynamics,
                Rotation2d.kZero,
                positionZero,
                Pose2d.kZero,
                0);

        OdometryUpdater ou = new OdometryUpdater(kinodynamics, gyro, history, () -> positions);
        ou.reset(Pose2d.kZero, 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(history, ou);

        // initial pose = 0
        verify(0, history.apply(0.00));

        // pose stays zero when updated at time zero
        // if we try to update zero, there's nothing to compare it to,
        // so we should just ignore this update.
        positions = positionZero;
        ou.update(0.0);
        verify(0.000, history.apply(0.00));
        verify(0.000, history.apply(0.01));
        verify(0.000, history.apply(0.02));

        // now vision says we're one meter away, so pose goes towards that
        vu.put(0.01, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        verify(0.167, history.apply(0.01));

        // if we had added this vision measurement here, it would have pulled the
        // estimate further
        // poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.015);
        // verify(0.305, poseEstimator.get());

        // wheels haven't moved, so the "odometry opinion" should be zero
        // but it's not, it's applied relative to the vision update, so there's no
        // change.
        positions = positionZero;
        ou.update(0.02);

        verify(0.000, history.apply(0.00));
        verify(0.167, history.apply(0.01));
        verify(0.167, history.apply(0.02));

        // wheels have moved 0.1m in +x, at t=0.04.
        // the "odometry opinion" should be 0.1 since the last odometry estimate was
        // 0, but instead odometry is applied relative to the latest estimate, which
        // was based on vision. so the actual odometry stddev is like *zero*.

        positions = position01;
        ou.update(0.04);

        verify(0.000, history.apply(0.00));
        verify(0.167, history.apply(0.02));
        verify(0.267, history.apply(0.04));

        // here's the delayed update from above, which moves the estimate to 0.305 and
        // then the odometry is applied on top of that, yielding 0.405.
        vu.put(0.015, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        verify(0.305, history.apply(0.015));
        // odometry thinks no motion at 0.02 so repeat the vision estimate here
        verify(0.305, history.apply(0.02));
        // odometry of 0.1 + the vision estimate from 0.02.
        verify(0.405, history.apply(0.04));

        // wheels are in the same position as the previous iteration,
        positions = position01;
        ou.update(0.06);
        verify(0.000, history.apply(0.00));
        verify(0.305, history.apply(0.02));
        verify(0.405, history.apply(0.04));
        verify(0.405, history.apply(0.06));
        verify(0.405, history.apply(0.08));

        // a little earlier than the previous estimate does nothing
        vu.put(0.014, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        // notices the vision input a bit earlier
        verify(0.305, history.apply(0.014));
        // but doesn't change this estimate since it's the same, and we're not moving,
        // we don't replay vision input
        // it would be better if two vision estimates pulled harder than one,
        // even if they come in out-of-order.
        verify(0.305, history.apply(0.015));
        verify(0.305, history.apply(0.02));
        verify(0.405, history.apply(0.04));
        verify(0.405, history.apply(0.06));

        // a little later than the previous estimate works normally.
        vu.put(0.016, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        verify(0.305, history.apply(0.014));
        verify(0.305, history.apply(0.015));
        // drag the pose towards the vision estimate a bit.
        verify(0.421, history.apply(0.016));
        verify(0.421, history.apply(0.02));
        verify(0.521, history.apply(0.04));
        verify(0.521, history.apply(0.06));

        // wheels not moving -> no change,
        positions = position01;
        ou.update(0.08);
        verify(0.000, history.apply(0.00));
        verify(0.421, history.apply(0.02));
        verify(0.521, history.apply(0.04));
        verify(0.521, history.apply(0.06));
        verify(0.521, history.apply(0.08));
    }

    @Test
    void minorWeirdness() {
        // weirdness with out-of-order vision updates

        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();

        double[] stateStdDevs = new double[] { 0.1, 0.1, 0.1 };
        double[] visionMeasurementStdDevs = new double[] { 0.5, 0.5, Double.MAX_VALUE };
        SwerveHistory history = new SwerveHistory(
                kinodynamics,
                Rotation2d.kZero,
                positionZero,
                Pose2d.kZero,
                0);

        OdometryUpdater ou = new OdometryUpdater(kinodynamics, gyro, history, () -> positions);
        positions = positionZero;
        ou.reset(Pose2d.kZero, 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(history, ou);

        // initial pose = 0
        verify(0.000, history.apply(0.00));
        verify(0.000, history.apply(0.02));
        verify(0.000, history.apply(0.04));
        verify(0.000, history.apply(0.06));
        verify(0.000, history.apply(0.08));

        // pose stays zero when updated at time zero
        // if we try to update zero, there's nothing to compare it to,
        // so we should just ignore this update.
        positions = positionZero;
        ou.update(0.0);
        verify(0.000, history.apply(0.00));
        verify(0.000, history.apply(0.02));
        verify(0.000, history.apply(0.04));
        verify(0.000, history.apply(0.06));
        verify(0.000, history.apply(0.08));

        // now vision says we're one meter away, so pose goes towards that
        vu.put(0.01, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        verify(0.167, history.apply(0.02));
        verify(0.167, history.apply(0.04));
        verify(0.167, history.apply(0.06));
        verify(0.167, history.apply(0.08));

        // if we had added this vision measurement here, it would have pulled the
        // estimate further
        // poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.015);
        // verify(0.305, poseEstimator.apply());

        // wheels haven't moved, so the "odometry opinion" should be zero
        // but it's not, it's applied relative to the vision update, so there's no
        // change.
        positions = positionZero;
        ou.update(0.02);
        verify(0.000, history.apply(0.00));
        verify(0.167, history.apply(0.02));
        verify(0.167, history.apply(0.04));
        verify(0.167, history.apply(0.06));
        verify(0.167, history.apply(0.08));

        // wheels have moved 0.1m in +x, at t=0.04.
        // the "odometry opinion" should be 0.1 since the last odometry estimate was
        // 0, but instead odometry is applied relative to the latest estimate, which
        // was based on vision. so the actual odometry stddev is like *zero*.

        positions = position01;
        ou.update(0.04);
        verify(0.000, history.apply(0.00));
        verify(0.167, history.apply(0.02));
        verify(0.267, history.apply(0.04));
        verify(0.267, history.apply(0.06));
        verify(0.267, history.apply(0.08));

        // here's the delayed update from above, which moves the estimate to 0.305 and
        // then the odometry is applied on top of that, yielding 0.405.
        vu.put(0.015, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        verify(0.305, history.apply(0.02));
        verify(0.405, history.apply(0.04));
        verify(0.405, history.apply(0.06));
        verify(0.405, history.apply(0.08));

        // wheels are in the same position as the previous iteration
        positions = position01;
        ou.update(0.06);
        verify(0.000, history.apply(0.00));
        verify(0.305, history.apply(0.02));
        verify(0.405, history.apply(0.04));
        verify(0.405, history.apply(0.06));
        verify(0.405, history.apply(0.08));

        // a little earlier than the previous estimate does nothing.
        vu.put(0.014, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        verify(0.305, history.apply(0.02));
        verify(0.405, history.apply(0.04));
        verify(0.405, history.apply(0.06));
        verify(0.405, history.apply(0.08));

        // a little later than the previous estimate works normally.
        vu.put(0.016, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        verify(0.421, history.apply(0.02));
        verify(0.521, history.apply(0.04));
        verify(0.521, history.apply(0.06));
        verify(0.521, history.apply(0.08));

        // wheels not moving -> no change
        positions = position01;
        ou.update(0.08);
        verify(0.000, history.apply(0.00));
        verify(0.421, history.apply(0.02));
        verify(0.521, history.apply(0.04));
        verify(0.521, history.apply(0.06));
        verify(0.521, history.apply(0.08));
    }

    @Test
    void test0105() {
        // this is the current (post-comp 2024) base case.
        // within a few frames, the estimate converges on the vision input.
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();

        double[] stateStdDevs = new double[] { 0.1, 0.1, 0.1 };
        double[] visionMeasurementStdDevs = new double[] { 0.5, 0.5, Double.MAX_VALUE };
        positions = positionZero;
        SwerveHistory history = new SwerveHistory(
                kinodynamics,
                Rotation2d.kZero,
                positionZero,
                Pose2d.kZero,
                0);

        OdometryUpdater ou = new OdometryUpdater(kinodynamics, gyro, history, () -> positions);
        positions = positionZero;
        ou.reset(Pose2d.kZero, 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(history, ou);

        verify(0.000, history.apply(0.00));
        verify(0.000, history.apply(0.02));
        verify(0.000, history.apply(0.04));
        verify(0.000, history.apply(0.06));
        verify(0.000, history.apply(0.08));

        positions = positionZero;
        ou.update(0);
        verify(0.000, history.apply(0.00));
        verify(0.000, history.apply(0.02));
        verify(0.000, history.apply(0.04));
        verify(0.000, history.apply(0.06));
        verify(0.000, history.apply(0.08));

        vu.put(0.02, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        verify(0.167, history.apply(0.02));
        verify(0.167, history.apply(0.04));
        verify(0.167, history.apply(0.06));
        verify(0.167, history.apply(0.08));

        vu.put(0.04, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        verify(0.167, history.apply(0.02));
        verify(0.305, history.apply(0.04));
        verify(0.305, history.apply(0.06));
        verify(0.305, history.apply(0.08));

        vu.put(0.06, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        verify(0.167, history.apply(0.02));
        verify(0.305, history.apply(0.04));
        verify(0.421, history.apply(0.06));
        verify(0.421, history.apply(0.08));

    }

    @Test
    void test0110() {
        // double vision stdev (r) -> slower convergence
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();

        double[] stateStdDevs = new double[] { 0.1, 0.1, 0.1 };
        double[] visionMeasurementStdDevs = new double[] { 1.0, 1.0, Double.MAX_VALUE };
        positions = positionZero;
        SwerveHistory history = new SwerveHistory(
                kinodynamics,
                Rotation2d.kZero,
                positionZero,
                Pose2d.kZero,
                0);

        OdometryUpdater ou = new OdometryUpdater(kinodynamics, gyro, history, () -> positions);
        ou.reset(Pose2d.kZero, 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(history, ou);

        verify(0.000, history.apply(0.00));
        verify(0.000, history.apply(0.02));
        verify(0.000, history.apply(0.04));
        verify(0.000, history.apply(0.06));
        verify(0.000, history.apply(0.08));

        positions = positionZero;
        ou.update(0);
        verify(0.000, history.apply(0.00));
        verify(0.000, history.apply(0.02));
        verify(0.000, history.apply(0.04));
        verify(0.000, history.apply(0.06));
        verify(0.000, history.apply(0.08));

        vu.put(0.02, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        verify(0.091, history.apply(0.02));
        verify(0.091, history.apply(0.04));
        verify(0.091, history.apply(0.06));
        verify(0.091, history.apply(0.08));

        vu.put(0.04, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        verify(0.091, history.apply(0.02));
        verify(0.173, history.apply(0.04));
        verify(0.173, history.apply(0.06));
        verify(0.173, history.apply(0.08));

        vu.put(0.06, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        verify(0.091, history.apply(0.02));
        verify(0.173, history.apply(0.04));
        verify(0.249, history.apply(0.06));
        verify(0.249, history.apply(0.08));
    }

    @Test
    void test00505() {
        // half odo stdev (q) -> slower convergence
        // the K is q/(q+qr) so it's q compared to r that matters.
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();

        double[] stateStdDevs = new double[] { 0.05, 0.05, 0.05 };
        double[] visionMeasurementStdDevs = new double[] { 0.5, 0.5, Double.MAX_VALUE };
        positions = positionZero;
        SwerveHistory history = new SwerveHistory(
                kinodynamics,
                Rotation2d.kZero,
                positionZero,
                Pose2d.kZero,
                0);

        OdometryUpdater ou = new OdometryUpdater(kinodynamics, gyro, history, () -> positions);
        ou.reset(Pose2d.kZero, 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(history, ou);

        verify(0.000, history.apply(0.00));
        verify(0.000, history.apply(0.02));
        verify(0.000, history.apply(0.04));
        verify(0.000, history.apply(0.06));
        verify(0.000, history.apply(0.08));

        positions = positionZero;
        ou.update(0);
        verify(0.000, history.apply(0.00));
        verify(0.000, history.apply(0.02));
        verify(0.000, history.apply(0.04));
        verify(0.000, history.apply(0.06));
        verify(0.000, history.apply(0.08));

        vu.put(0.02, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        verify(0.091, history.apply(0.02));
        verify(0.091, history.apply(0.04));
        verify(0.091, history.apply(0.06));
        verify(0.091, history.apply(0.08));

        vu.put(0.04, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        verify(0.091, history.apply(0.02));
        verify(0.173, history.apply(0.04));
        verify(0.173, history.apply(0.06));
        verify(0.173, history.apply(0.08));

        vu.put(0.06, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        verify(0.091, history.apply(0.02));
        verify(0.173, history.apply(0.04));
        verify(0.249, history.apply(0.06));
        verify(0.249, history.apply(0.08));
    }

    @Test
    void reasonable() {
        // stdev that actually make sense
        // actual odometry error is very low
        // measured camera error is something under 10 cm
        // these yield much slower convergence, maybe too slow? try and see.
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();

        double[] stateStdDevs = new double[] { 0.001, 0.001, 0.01 };
        double[] visionMeasurementStdDevs = new double[] { 0.1, 0.1, Double.MAX_VALUE };
        SwerveHistory history = new SwerveHistory(
                kinodynamics,
                Rotation2d.kZero,
                positionZero,
                Pose2d.kZero,
                0);

        OdometryUpdater ou = new OdometryUpdater(kinodynamics, gyro, history, () -> positions);
        positions = positionZero;
        ou.reset(Pose2d.kZero, 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(history, ou);

        verify(0.000, history.apply(0.00));
        verify(0.000, history.apply(0.02));
        verify(0.000, history.apply(0.04));
        verify(0.000, history.apply(0.06));
        verify(0.000, history.apply(0.08));

        positions = positionZero;
        ou.update(0);
        verify(0.000, history.apply(0.00));
        verify(0.000, history.apply(0.02));
        verify(0.000, history.apply(0.04));
        verify(0.000, history.apply(0.06));
        verify(0.000, history.apply(0.08));

        vu.put(0.02, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        verify(0.010, history.apply(0.02));
        verify(0.010, history.apply(0.04));
        verify(0.010, history.apply(0.06));
        verify(0.010, history.apply(0.08));

        vu.put(0.04, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        verify(0.010, history.apply(0.02));
        verify(0.020, history.apply(0.04));
        verify(0.020, history.apply(0.06));
        verify(0.020, history.apply(0.08));

        vu.put(0.06, visionRobotPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        verify(0.000, history.apply(0.00));
        verify(0.010, history.apply(0.02));
        verify(0.020, history.apply(0.04));
        verify(0.029, history.apply(0.06));
        verify(0.029, history.apply(0.08));

    }

    ////////////////////////////////////////
    //
    // tests below are from WPILib
    //
    //

    // used below
    State groundTruthState = new State();
    Random rand = new Random(3538);
    Trajectory trajectory = new Trajectory();

    @Test
    void testAccuracyFacingTrajectory() {
        Gyro gyro = new Gyro() {
            @Override
            public Rotation2d getYawNWU() {
                return groundTruthState.poseMeters
                        .getRotation()
                        .plus(new Rotation2d(rand.nextGaussian() * 0.05))
                        .minus(trajectory.getInitialPose().getRotation());
            }

            @Override
            public double getYawRateNWU() {
                return 0.0;
            }

            @Override
            public Rotation2d getPitchNWU() {
                return null;
            }

            @Override
            public Rotation2d getRollNWU() {
                return null;
            }

            @Override
            public void periodic() {
            }
        };
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forWPITest();

        var fl = new SwerveModulePosition100();
        var fr = new SwerveModulePosition100();
        var bl = new SwerveModulePosition100();
        var br = new SwerveModulePosition100();

        double[] stateStdDevs = new double[] { 0.1, 0.1, 0.1 };
        double[] visionMeasurementStdDevs = new double[] { 0.5, 0.5, 0.5 };
        var estimator = new SwerveHistory(
                kinodynamics,
                Rotation2d.kZero,
                positionZero,
                Pose2d.kZero,
                0); // zero initial time

        OdometryUpdater ou = new OdometryUpdater(kinodynamics, gyro, estimator, () -> positions);
        positions = new SwerveModulePositions(fl, fr, bl, br);
        ou.reset(
                Rotation2d.kZero,
                new Pose2d(),
                0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(estimator, ou);

        trajectory = TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(0, 0, Rotation2d.fromDegrees(45)),
                        new Pose2d(3, 0, Rotation2d.fromDegrees(-90)),
                        new Pose2d(0, 0, Rotation2d.fromDegrees(135)),
                        new Pose2d(-3, 0, Rotation2d.fromDegrees(-90)),
                        new Pose2d(0, 0, Rotation2d.fromDegrees(45))),
                new TrajectoryConfig(2, 2));

        final Pose2d endingPose = new Pose2d(0, 0, Rotation2d.fromDegrees(45));

        positions = new SwerveModulePositions(
                new SwerveModulePosition100(),
                new SwerveModulePosition100(),
                new SwerveModulePosition100(),
                new SwerveModulePosition100());

        // new starting pose here, so we don't actually use the earlier initial pose

        ou.reset(Rotation2d.kZero,
                trajectory.getInitialPose(),
                0);

        double t = 0.0;

        final TreeMap<Double, Pose2d> visionUpdateQueue = new TreeMap<>();

        double maxError = Double.NEGATIVE_INFINITY;
        double errorSum = 0;

        while (t <= trajectory.getTotalTimeSeconds()) {
            groundTruthState = trajectory.sample(t);

            // We are due for a new vision measurement if it's been `visionUpdateRate`
            // seconds since the last vision measurement
            if (visionUpdateQueue.isEmpty() || visionUpdateQueue.lastKey() + 0.1 < t) {
                Pose2d newVisionPose = ((Function<State, Pose2d>) state -> state.poseMeters)
                        .apply(groundTruthState)
                        .plus(
                                new Transform2d(
                                        new Translation2d(rand.nextGaussian() * 0.1, rand.nextGaussian() * 0.1),
                                        new Rotation2d(rand.nextGaussian() * 0.05)));

                visionUpdateQueue.put(t, newVisionPose);
            }

            // We should apply the oldest vision measurement if it has been
            // `visionUpdateDelay` seconds since it was measured
            if (!visionUpdateQueue.isEmpty() && visionUpdateQueue.firstKey() + 0.25 < t) {
                var visionEntry = visionUpdateQueue.pollFirstEntry();
                vu.put(
                        visionEntry.getKey(),
                        visionEntry.getValue(),
                        stateStdDevs,
                        visionMeasurementStdDevs);
            }

            ChassisSpeeds chassisSpeeds = ((Function<State, ChassisSpeeds>) state -> new ChassisSpeeds(
                    state.velocityMetersPerSecond,
                    0,
                    state.velocityMetersPerSecond * state.curvatureRadPerMeter)).apply(groundTruthState);

            SwerveModuleStates moduleStates = kinodynamics.getKinematics()
                    .toSwerveModuleStates(SwerveKinodynamics.discretize(chassisSpeeds, 0.02));
            SwerveModuleState100[] moduleStatesAll = moduleStates.all();
            SwerveModulePosition100[] positionsAll = positions.all();
            for (int i = 0; i < moduleStatesAll.length; i++) {
                positionsAll[i].distanceMeters += moduleStatesAll[i].speedMetersPerSecond()
                        * (1 - rand.nextGaussian() * 0.05)
                        * 0.02;
                Optional<Rotation2d> angle = moduleStatesAll[i].angle();
                double noise = rand.nextGaussian() * 0.005;
                if (angle.isPresent()) {
                    positionsAll[i].unwrappedAngle = Optional.of(
                            new Rotation2d(angle.get().getRadians() + noise));
                } else {
                    positionsAll[i].unwrappedAngle = Optional.empty();
                }
            }

            ou.update(t);
            ModelR3 xHat = estimator.apply(t);

            double error = groundTruthState.poseMeters.getTranslation().getDistance(xHat.pose().getTranslation());
            if (error > maxError) {
                maxError = error;
            }
            errorSum += error;

            if (DEBUG) {
                System.out.printf("t %5.3f refX %5.3f refY %5.3f xhatX %5.3f xhatY %5.3f\n",
                        t,
                        groundTruthState.poseMeters.getX(),
                        groundTruthState.poseMeters.getY(),
                        xHat.pose().getX(),
                        xHat.pose().getY());
            }

            t += 0.02;

        }

        Pose2d estimatedEndingPose = estimator.apply(t).pose();
        assertEquals(
                endingPose.getX(), estimatedEndingPose.getX(), 0.08, "Incorrect Final X");
        assertEquals(
                endingPose.getY(), estimatedEndingPose.getY(), 0.08, "Incorrect Final Y");
        assertEquals(
                endingPose.getRotation().getRadians(),
                estimatedEndingPose.getRotation().getRadians(),
                0.15,
                "Incorrect Final Theta");

        if (true) {
            assertEquals(
                    0.0, errorSum / (trajectory.getTotalTimeSeconds() / 0.02), 0.07, "Incorrect mean error");
            assertEquals(0.0, maxError, 0.2, "Incorrect max error");
        }

    }

    @Test
    void testSimultaneousVisionMeasurements() {
        // This tests for multiple vision measurements appled at the same time. The
        // expected behavior is that all measurements affect the estimated pose. The
        // alternative result is that only one vision measurement affects the outcome.
        // If that were the case, after 1000 measurements, the estimated pose would
        // converge to that measurement.

        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forWPITest();
        Gyro gyro = new MockGyro();

        final SwerveModulePosition100 fl = new SwerveModulePosition100();
        final SwerveModulePosition100 fr = new SwerveModulePosition100();
        final SwerveModulePosition100 bl = new SwerveModulePosition100();
        final SwerveModulePosition100 br = new SwerveModulePosition100();

        double[] stateStdDevs = new double[] { 0.1, 0.1, 0.1 };
        double[] visionMeasurementStdDevs = new double[] { 0.9, 0.9, 0.9 };
        var estimator = new SwerveHistory(
                kinodynamics,
                Rotation2d.kZero,
                positionZero,
                Pose2d.kZero,
                0); // zero initial time

        OdometryUpdater ou = new OdometryUpdater(kinodynamics, gyro, estimator, () -> positions);
        positions = new SwerveModulePositions(fl, fr, bl, br);
        ou.reset(new Pose2d(1, 2, Rotation2d.fromDegrees(270)), 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(estimator, ou);

        ou.update(0);

        var visionMeasurements = new Pose2d[] {
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(3, 1, Rotation2d.fromDegrees(90)),
                new Pose2d(2, 4, Rotation2d.fromRadians(180)),
        };

        for (int i = 0; i < 1000; i++) {
            for (var measurement : visionMeasurements) {
                vu.put(0.00, measurement, stateStdDevs, visionMeasurementStdDevs);
            }
        }

        for (var measurement : visionMeasurements) {
            // at time zero the whole time
            Pose2d estimatedPose = estimator.apply(0).pose();
            var dx = Math.abs(measurement.getX() - estimatedPose.getX());
            var dy = Math.abs(measurement.getY() - estimatedPose.getY());
            var dtheta = Math.abs(
                    measurement.getRotation().getDegrees()
                            - estimatedPose.getRotation().getDegrees());

            assertTrue(dx > 0.08 || dy > 0.08 || dtheta > 0.08);
        }
    }

    @Test
    void testDiscardsOldVisionMeasurements() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forWPITest();
        Gyro gyro = new MockGyro();

        var estimator = new SwerveHistory(
                kinodynamics,
                Rotation2d.kZero,
                positionZero,
                Pose2d.kZero,
                0); // zero initial time

        OdometryUpdater ou = new OdometryUpdater(kinodynamics, gyro, estimator, () -> positions);
        positions = new SwerveModulePositions(
                new SwerveModulePosition100(),
                new SwerveModulePosition100(),
                new SwerveModulePosition100(),
                new SwerveModulePosition100());
        ou.reset(Pose2d.kZero, 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(estimator, ou);

        double time = 0;

        // Add enough measurements to fill up the buffer
        for (; time < 4; time += 0.02) {
            ou.update(time);
        }

        Pose2d odometryPose = estimator.apply(time).pose();

        // Apply a vision measurement made 3 seconds ago
        // This test passes if this does not cause a ConcurrentModificationException.
        vu.put(
                1,
                new Pose2d(new Translation2d(10, 10), new Rotation2d(0.1)),
                new double[] { 0.1, 0.1, 0.1 },
                new double[] { 0.1, 0.1, 0.1 });

        Pose2d visionPose = estimator.apply(time).pose();

        assertEquals(odometryPose.getX(), visionPose.getX(), DELTA);
        assertEquals(odometryPose.getY(), visionPose.getY(), DELTA);
        assertEquals(
                odometryPose.getRotation().getRadians(),
                visionPose.getRotation().getRadians(), DELTA);
    }
}
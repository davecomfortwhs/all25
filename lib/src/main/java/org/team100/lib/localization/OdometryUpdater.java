package org.team100.lib.localization;

import java.util.Map;
import java.util.Map.Entry;
import java.util.function.Supplier;

import org.team100.lib.coherence.Takt;
import org.team100.lib.geometry.GlobalDeltaR3;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.gyro.Gyro;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.motion.swerve.module.state.SwerveModulePositions;
import org.team100.lib.state.ModelR3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Updates SwerveModelHistory with new odometry by selecting the most-recent
 * pose
 * and applying the pose delta represented by the odometry.
 * 
 * Uses the gyro angle and rate instead of the odometry-derived values, because
 * the gyro is more accurate.
 * 
 * Manages the gyro offset.
 * 
 * Note we use methods on the specific history implementation; the interface
 * won't work here.
 */
public class OdometryUpdater {
    private static final boolean DEBUG = false;

    private final SwerveKinodynamics m_kinodynamics;
    private final Gyro m_gyro;
    private final SwerveHistory m_history;
    private final Supplier<SwerveModulePositions> m_positions;

    private Rotation2d m_gyroOffset;

    public OdometryUpdater(
            SwerveKinodynamics kinodynamics,
            Gyro gyro,
            SwerveHistory estimator,
            Supplier<SwerveModulePositions> positions) {
        m_kinodynamics = kinodynamics;
        m_gyro = gyro;
        m_history = estimator;
        m_positions = positions;
    }

    Rotation2d getGyroOffset() {
        return m_gyroOffset;
    }

    /**
     * Put a new state estimate based on gyro and wheel data, from the suppliers
     * passed to the constructor. There is no history replay here, though it won't
     * fail if you give it out-of-order input.
     * 
     * It Samples the history before the specified time, and records a new pose
     * based on the difference in wheel positions between the sample and the
     * specified positions.
     * 
     * The gyro angle overrides the odometry-derived gyro measurement, and
     * the gyro rate overrides the rate derived from the difference to the previous
     * state.
     */
    public void update() {
        update(Takt.get());
    }

    /** For testing. */
    void update(double timestamp) {
        put(timestamp, m_gyro.getYawNWU(), m_gyro.getYawRateNWU(), m_positions.get());
    }

    /**
     * Empty the history, reset the gyro offset, and add the given measurements.
     * Uses the module position supplier passed to the constructor.
     */
    public void reset(Pose2d pose) {
        reset(m_gyro.getYawNWU(), pose, Takt.get());
    }

    /** For testing. */
    public void reset(Pose2d pose, double timestampSeconds) {
        m_gyroOffset = pose.getRotation().minus(m_gyro.getYawNWU());
        m_history.reset(m_positions.get(), pose, timestampSeconds);
    }

    /**
     * For testing.
     */
    void reset(
            Rotation2d gyroAngle,
            Pose2d pose,
            double timestampSeconds) {
        m_gyroOffset = pose.getRotation().minus(gyroAngle);
        m_history.reset(m_positions.get(), pose, timestampSeconds);
    }

    ////////////////////////////////////////////////////

    private void put(
            double currentTimeS,
            Rotation2d gyroAngleRadNWU,
            double gyroRateRad_SNWU,
            SwerveModulePositions wheelPositions) {

        // the entry right before this one, the basis for integration.
        Entry<Double, InterpolationRecord> lowerEntry = m_history.lowerEntry(
                currentTimeS);

        if (lowerEntry == null) {
            // System.out.println("lower entry is null");
            // We're at the beginning. There's nothing to apply the wheel position delta to.
            // This should never happen.
            return;
        }

        double dt = currentTimeS - lowerEntry.getKey();
        InterpolationRecord value = lowerEntry.getValue();
        ModelR3 previousState = value.m_state;
        if (DEBUG) {
            System.out.printf("previous x %.6f y %.6f\n", previousState.pose().getX(), previousState.pose().getY());
        }

        SwerveModuleDeltas modulePositionDelta = SwerveModuleDeltas.modulePositionDelta(
                value.m_wheelPositions,
                wheelPositions);
        if (DEBUG) {
            System.out.printf("modulePositionDelta %s\n", modulePositionDelta);
        }

        Twist2d twist = m_kinodynamics.getKinematics().toTwist2d(modulePositionDelta);
        if (DEBUG) {
            System.out.printf("twist x %.6f y %.6f theta %.6f\n", twist.dx, twist.dy, twist.dtheta);
        }
        // replace the twist dtheta with one derived from the current
        // pose angle based on the gyro (which is more accurate)

        Rotation2d angle = gyroAngleRadNWU.plus(m_gyroOffset);
        if (DEBUG) {
            System.out.printf("angle %.6f\n", angle.getRadians());
        }
        twist.dtheta = angle.minus(previousState.pose().getRotation()).getRadians();

        Pose2d newPose = previousState.pose().exp(twist);
        if (DEBUG) {
            System.out.printf("new pose x %.6f y %.6f\n", newPose.getX(), newPose.getY());
        }

        // this is the backward finite difference velocity from odometry
        GlobalDeltaR3 odoVelo = GlobalDeltaR3.delta(
                previousState.pose(), newPose)
                .div(dt);

        // use the gyro rate instead of the odometry-derived rate
        GlobalVelocityR3 velocity = new GlobalVelocityR3(
                odoVelo.getX(),
                odoVelo.getY(),
                gyroRateRad_SNWU);

        ModelR3 swerveState = new ModelR3(newPose, velocity);

        m_history.put(currentTimeS, swerveState, wheelPositions);
    }

    /** Replay odometry after the sample time. */
    void replay(double timestamp) {
        // Note the exclusive tailmap: we don't see the entry at timestamp.
        for (Map.Entry<Double, InterpolationRecord> entry : m_history.exclusiveTailMap(timestamp).entrySet()) {
            double entryTimestampS = entry.getKey();
            InterpolationRecord value = entry.getValue();

            // this is what the gyro must have been given the pose and offset
            // note that stale gyro offsets never occur, because the gyro offset is
            // reset at the same time the buffer is emptied.
            Rotation2d entryGyroAngle = value.m_state.pose().getRotation().minus(m_gyroOffset);
            double entryGyroRate = value.m_state.theta().v();
            SwerveModulePositions wheelPositions = value.m_wheelPositions;

            put(entryTimestampS, entryGyroAngle, entryGyroRate, wheelPositions);
        }
    }

}

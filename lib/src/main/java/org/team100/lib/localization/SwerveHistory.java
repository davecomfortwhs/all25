package org.team100.lib.localization;

import java.util.Map.Entry;
import java.util.SortedMap;
import java.util.function.DoubleFunction;

import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.module.state.SwerveModulePositions;
import org.team100.lib.state.ModelR3;
import org.team100.lib.util.TimeInterpolatableBuffer100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * History is just a container, in fact the implementation is little more than a
 * wrapper around TimeInterpolatableBuffer.
 * 
 * The history always has *something* in it, even the initial zero pose.
 * 
 * There are no dependencies managed here; for that, use SwerveModelEstimate.
 * 
 * Note this should only be used from within the localization package.
 * 
 * Other SwerveModel consumers should use SwerveModelEstimate.
 */
public class SwerveHistory implements DoubleFunction<ModelR3> {
    /**
     * The buffer only needs to be long enough to catch stale-but-still-helpful
     * vision updates.
     * 
     * The current Raspberry Pi cameras seem to be able to provide frames to RoboRIO
     * code with about 75-100 ms latency. There will never be a vision update
     * older than about 200 ms.
     */
    private static final double BUFFER_DURATION = 0.2;

    private final SwerveKinodynamics m_kinodynamics;
    private final TimeInterpolatableBuffer100<InterpolationRecord> m_poseBuffer;

    public SwerveHistory(
            SwerveKinodynamics kinodynamics,
            Rotation2d gyroAngle,
            SwerveModulePositions modulePositions,
            Pose2d initialPoseMeters,
            double timestampSeconds) {
        m_kinodynamics = kinodynamics;
        m_poseBuffer = new TimeInterpolatableBuffer100<>(
                BUFFER_DURATION,
                timestampSeconds,
                new InterpolationRecord(
                        m_kinodynamics.getKinematics(),
                        new ModelR3(
                                initialPoseMeters,
                                new GlobalVelocityR3(0, 0, 0)),
                        modulePositions));
    }

    /**
     * Sample the state estimate buffer.
     */
    @Override
    public ModelR3 apply(double timestampSeconds) {
        return m_poseBuffer.get(timestampSeconds).m_state;
    }

    /** Empty the buffer and add the given measurements. */
    void reset(
            SwerveModulePositions modulePositions,
            Pose2d pose,
            double timestampSeconds) {
        // empty the buffer and add the current pose
        m_poseBuffer.reset(
                timestampSeconds,
                new InterpolationRecord(
                        m_kinodynamics.getKinematics(),
                        new ModelR3(pose, new GlobalVelocityR3(0, 0, 0)),
                        modulePositions));
    }

    //////////////////////////////////////////////////
    // methods below are for history maintenance

    /**
     * timestamp in seconds
     */
    void put(
            double timestamp,
            ModelR3 model,
            SwerveModulePositions positions) {
        m_poseBuffer.put(
                timestamp,
                new InterpolationRecord(
                        m_kinodynamics.getKinematics(),
                        model,
                        positions));
    }

    Entry<Double, InterpolationRecord> lowerEntry(double timestamp) {
        return m_poseBuffer.lowerEntry(timestamp);
    }

    InterpolationRecord getRecord(double timestamp) {
        return m_poseBuffer.get(timestamp);
    }

    boolean tooOld(double timestamp) {
        return m_poseBuffer.tooOld(timestamp);
    }

    SortedMap<Double, InterpolationRecord> exclusiveTailMap(double timestamp) {
        return m_poseBuffer.tailMap(timestamp, false);
    }

    int size() {
        return m_poseBuffer.size();
    }

    double lastKey() {
        return m_poseBuffer.lastKey();
    }

}

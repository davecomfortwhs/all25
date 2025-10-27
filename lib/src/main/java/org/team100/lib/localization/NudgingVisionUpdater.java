package org.team100.lib.localization;

import org.team100.lib.state.ModelR3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Updates SwerveModelHistory with any vision input, by interpolating to find a
 * pose for the vision timestamp, nudging that pose towards the vision
 * measurement, and then asking the odometry updater to replay all the later
 * odometry.
 * 
 * The "nudging" here is essentially just a weighted average; you provide the
 * weights you want at update time.
 */
public class NudgingVisionUpdater implements VisionUpdater {

    private final SwerveHistory m_history;
    /** For replay. */
    private final OdometryUpdater m_odometryUpdater;

    public NudgingVisionUpdater(
            SwerveHistory history,
            OdometryUpdater odometryUpdater) {
        m_history = history;
        m_odometryUpdater = odometryUpdater;
    }

    /**
     * Put a new state estimate based on the supplied pose. If not current,
     * subsequent wheel updates are replayed.
     */
    @Override
    public void put(
            double timestampS,
            Pose2d measurement,
            double[] stateSigma,
            double[] visionSigma) {

        // Skip too-old measurement
        if (m_history.tooOld(timestampS)) {
            return;
        }

        // Sample the history at the measurement time.
        InterpolationRecord sample = m_history.getRecord(timestampS);

        // If there is a sample, nudge it towards the measurement.
        Pose2d nudged = nudge(
                sample.m_state.pose(), measurement, stateSigma, visionSigma);
        m_history.put(
                timestampS,
                new ModelR3(nudged, sample.m_state.velocity()),
                sample.m_wheelPositions);
        m_odometryUpdater.replay(timestampS);
    }

    /////////////////////////////////////////

    /**
     * Nudge the sample towards the measurement.
     */
    static Pose2d nudge(
            Pose2d sample,
            Pose2d measurement,
            double[] stateSigma,
            double[] visionSigma) {
        // Step 2: Measure the twist between the odometry pose and the vision pose.
        Twist2d twist = sample.log(measurement);
        // Step 4: Discount the twist based on the sigmas relative to each other
        Twist2d scaledTwist = getScaledTwist(stateSigma, visionSigma, twist);
        return sample.exp(scaledTwist);
    }

    static Twist2d getScaledTwist(
            double[] stateSigma,
            double[] visionSigma,
            Twist2d twist) {
        // discount the vision update by this factor.
        final double[] K = getK(stateSigma, visionSigma);
        Twist2d scaledTwist = new Twist2d(
                K[0] * twist.dx,
                K[1] * twist.dy,
                K[2] * twist.dtheta);
        return scaledTwist;
    }

    static double[] getK(double[] stateSigma, double[] visionSigma) {
        return new double[] {
                mix(Math.pow(stateSigma[0], 2), Math.pow(visionSigma[0], 2)),
                mix(Math.pow(stateSigma[1], 2), Math.pow(visionSigma[1], 2)),
                mix(Math.pow(stateSigma[2], 2), Math.pow(visionSigma[2], 2))
        };
    }

    /**
     * Given q and r stddev's, what mixture should that yield?
     * This is the "closed form Kalman gain for continuous Kalman filter with A = 0
     * and C = I. See wpimath/algorithms.md." ... but really it's just a mixer.
     * 
     * @param q state variance
     * @param r vision variance
     */
    static double mix(final double q, final double r) {
        if (q == 0.0)
            return 0.0;
        return q / (q + Math.sqrt(q * r));
    }

}

package org.team100.lib.localization;

import java.util.Optional;
import java.util.function.DoubleFunction;

import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.EnumLogger;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.logging.LoggerFactory.Transform3dLogger;
import org.team100.lib.network.CameraReader;
import org.team100.lib.state.ModelR3;
import org.team100.lib.util.TrailingHistory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Extracts robot pose estimates from camera observations of AprilTags.
 * 
 * Note this class depends only on the state *history*, not on the coherent sate
 * *estimate*. The camera input doesn't require fresh odometry, it modifies the
 * past (and replays up to the present).
 */
public class AprilTagRobotLocalizer extends CameraReader<Blip24> {
    private static final boolean DEBUG = false;
    /** Maximum age of the sights we publish for diagnosis. */
    private static final double HISTORY_DURATION = 1.0;

    /**
     * If the tag is closer than this threshold, then the camera's estimate of tag
     * rotation might be more accurate than the gyro, so we use the camera's
     * estimate of tag rotation to update the robot pose. If the tag is further away
     * than this, then the camera-derived rotation is probably less accurate than
     * the gyro, so we use the gyro instead.
     * 
     * Set this to zero to disable tag-derived rotation and always use the gyro.
     * 
     * Set this to some large number (e.g. 100) to disable gyro-derived rotation and
     * always use the camera.
     */
    private static final double TAG_ROTATION_BELIEF_THRESHOLD_M = 0;
    /** Discard results further than this from the previous one. */
    private static final double VISION_CHANGE_TOLERANCE_M = 0.1;
    // private static final double VISION_CHANGE_TOLERANCE_M = 1;

    /** this is the default value which, in hindsight, seems ridiculously high. */
    private static final double[] defaultStateStdDevs = new double[] {
            0.1,
            0.1,
            0.1 };
    /**
     * This value is tuned so that errors scale at 0.2x per second. See
     * SwerveDrivePoseEstimator100Test::testFirmerNudge.
     */
    static final double[] tightStateStdDevs = new double[] {
            0.001,
            0.001,
            0.1 };

    private final DoubleFunction<ModelR3> m_history;
    private final VisionUpdater m_visionUpdater;
    private final AprilTagFieldLayoutWithCorrectOrientation m_layout;

    /**
     * The apparent position of tags we see: this can be shown in AdvantageScope
     * using the Vision Target feature. The apparent position should match the
     * actual position, if the cameras are calibrated correctly. Note this involves
     * matching the frame timestamp with the pose history timestamp, so if the blip
     * source timestamp is wrong (as it is at the moment in the simulated tag
     * detector) then these positions will be a little bit wrong.
     */
    private final StructArrayPublisher<Pose3d> m_pub_tags;
    /** Just tags we use for pose estimation. */
    private final StructArrayPublisher<Pose3d> m_pub_used_tags;

    /**
     * The pose we derive from each sighting, so we can see it in AdvantageScope's
     * map, which can't understand our usual Pose2dLogger's output.
     */
    private final StructPublisher<Pose2d> m_pub_pose;

    // LOGGERS
    private final EnumLogger m_log_alliance;
    private final DoubleLogger m_log_heedRadius;
    private final BooleanLogger m_log_using_gyro;
    private final DoubleLogger m_log_tag_error;
    private final Pose2dLogger m_log_pose;
    /** For calibration. */
    private final Transform3dLogger m_log_tag_in_camera;

    /**
     * The difference between the current instant and the instant of the blip,
     * including our magic correction, i.e. this is the time we look up in the pose
     * buffer.
     */
    private final DoubleLogger m_log_lag;

    // Remember the previous vision-based pose estimate, so we can measure the
    // distance between consecutive updates, and ignore too-far updates.
    private Pose2d m_prevPose;

    private double m_latestTime = 0;

    /** use tags closer than this; ignore tags further than this. */
    private double m_heedRadiusM = 3.5;

    /**
     * Accumulates all tags we receive in each cycle, whether we use them or not.
     */
    private final TrailingHistory<Pose3d> m_allTags;
    /**
     * Just the tags we use for pose estimation, i.e. not ones that are too far
     * away.
     */
    private final TrailingHistory<Pose3d> m_usedTags;

    /**
     * @param parent        logger
     * @param layout        map of apriltags
     * @param history       f(timestamp) = swerve state, use SwerveModelHistory.
     * @param visionUpdater mutates history
     */
    public AprilTagRobotLocalizer(
            LoggerFactory parent,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            DoubleFunction<ModelR3> history,
            VisionUpdater visionUpdater) {
        super("vision", "blips", StructBuffer.create(Blip24.struct));
        LoggerFactory child = parent.type(this);
        m_layout = layout;
        m_history = history;
        m_visionUpdater = visionUpdater;
        m_allTags = new TrailingHistory<>(HISTORY_DURATION);
        m_usedTags = new TrailingHistory<>(HISTORY_DURATION);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_pub_tags = inst.getStructArrayTopic("tags", Pose3d.struct).publish();
        m_pub_used_tags = inst.getStructArrayTopic("used tags", Pose3d.struct).publish();
        m_pub_pose = inst.getStructTopic("pose", Pose2d.struct).publish();

        m_log_alliance = child.enumLogger(Level.TRACE, "alliance");
        m_log_heedRadius = child.doubleLogger(Level.TRACE, "heed radius");
        m_log_using_gyro = child.booleanLogger(Level.TRACE, "rotation source");
        m_log_tag_error = child.doubleLogger(Level.TRACE, "tag error");
        m_log_pose = child.pose2dLogger(Level.TRACE, "pose");
        m_log_tag_in_camera = child.transform3dLogger(Level.TRACE, "tag in camera");
        m_log_lag = child.doubleLogger(Level.TRACE, "lag");
    }

    /**
     * The age of the last pose estimate, in microseconds.
     * The caller could use this to, say, indicate tag visibility.
     */
    public double getPoseAgeSec() {
        double now = Takt.get();
        return now - m_latestTime;
    }

    @Override
    protected void perValue(
            Transform3d cameraOffset,
            double valueTimestamp,
            Blip24[] blips) {
        estimateRobotPose(
                cameraOffset,
                blips,
                valueTimestamp,
                DriverStation.getAlliance());
    }

    @Override
    protected void finishUpdate() {
        m_pub_tags.set(m_allTags.getAll().toArray(new Pose3d[0]));
        m_pub_used_tags.set(m_usedTags.getAll().toArray(new Pose3d[0]));
    }

    /**
     * Tags outside this radius are ignored.
     */
    public void setHeedRadiusM(double heedRadiusM) {
        m_heedRadiusM = heedRadiusM;
    }

    /**
     * Compute the robot pose and put it in the pose estimator.
     * 
     * @param cameraOffset   Camera pose in robot coordinates
     * @param blips          The targets in the current camera frame
     * @param valueTimestamp Camera frame timestamp
     * @param optAlliance    From the driver station, it's here to make testing
     *                       easier.
     */
    void estimateRobotPose(
            Transform3d cameraOffset,
            Blip24[] blips,
            double valueTimestamp,
            Optional<Alliance> optAlliance) {

        // Vasili added this extra delay after some experimentation that he should
        // describe here.
        final double IMPORTANT_MAGIC_NUMBER = 0.027;
        double correctedTimestamp = valueTimestamp - IMPORTANT_MAGIC_NUMBER;

        // this seems to always be 1. ????
        m_log_lag.log(() -> Takt.get() - correctedTimestamp);

        if (!optAlliance.isPresent()) {
            // this happens on startup
            if (DEBUG)
                System.out.println("WARNING: VisionDataProvider24: Alliance is not present!");
            return;
        }
        Alliance alliance = optAlliance.get();
        m_log_alliance.log(() -> alliance);
        m_log_heedRadius.log(() -> m_heedRadiusM);

        // The pose from the frame timestamp.
        // note this pulls from the *old history* not the *odometry-updated history*
        // because we don't care about the latest odometry update: we're trying to
        // affect the history from several cycles ago -- and replaying that. it's ok for
        // new odometry to be the last thing.
        final Pose2d historicalPose = m_history.apply(correctedTimestamp).pose();

        // The gyro rotation for the frame timestamp
        final Rotation2d gyroRotation = historicalPose.getRotation();
        if (DEBUG) {
            System.out.printf("gyro rotation %f\n", gyroRotation.getRadians());
        }

        for (int i = 0; i < blips.length; ++i) {
            Blip24 blip = blips[i];

            if (DEBUG) {
                Translation3d t = blip.getRawPose().getTranslation();
                Rotation3d r = blip.getRawPose().getRotation();
                System.out.printf("blip raw pose %d X %5.2f Y %5.2f Z %5.2f R %5.2f P %5.2f Y %5.2f\n",
                        blip.getId(), t.getX(), t.getY(), t.getZ(), r.getX(), r.getY(), r.getZ());
            }

            // Look up the pose of the tag in the field frame.
            Optional<Pose3d> tagInFieldCoordsOptional = m_layout.getTagPose(alliance, blip.getId());
            if (!tagInFieldCoordsOptional.isPresent()) {
                // This shouldn't happen, but it does.
                System.out.printf("WARNING: VisionDataProvider24: no tag for id %d\n", blip.getId());
                continue;
            }

            // Field-to-tag, canonical pose from the JSON file
            final Pose3d tagInField = tagInFieldCoordsOptional.get();

            // Camera-to-tag, as it appears in the camera frame.
            Transform3d blipTransform = blip.blipToTransform();
            m_log_tag_in_camera.log(() -> blipTransform);

            Transform3d tagInCamera = blipTransform;

            if (DEBUG) {
                // This is used for camera offset calibration. Place a tag at a known position,
                // observe the offset, and add it to Camera.java, inverted.
                Transform3d tagInRobot = cameraOffset.plus(tagInCamera);
                System.out.printf("tagInRobot id %d X %5.2f Y %5.2f Z %5.2f R %5.2f P %5.2f Y %5.2f\n",
                        blip.getId(), tagInRobot.getTranslation().getX(), tagInRobot.getTranslation().getY(),
                        tagInRobot.getTranslation().getZ(), tagInRobot.getRotation().getX(),
                        tagInRobot.getRotation().getY(), tagInRobot.getRotation().getZ());
            }

            if (tagInCamera.getTranslation().getNorm() > TAG_ROTATION_BELIEF_THRESHOLD_M) {
                // If the tag is further than the threshold, replace the tag rotation with
                // a rotation derived from the gyro, if available.
                m_log_using_gyro.log(() -> true);
                tagInCamera = PoseEstimationHelper.tagInCamera(
                        cameraOffset,
                        tagInField,
                        tagInCamera,
                        new Rotation3d(gyroRotation));
            } else {
                m_log_using_gyro.log(() -> false);
            }

            extralog(
                    correctedTimestamp,
                    historicalPose,
                    cameraOffset,
                    tagInCamera,
                    tagInField);

            // Estimate of robot pose.
            Pose3d pose3d = PoseEstimationHelper.robotInField(
                    cameraOffset,
                    tagInField,
                    tagInCamera);

            // Robot in field frame. Use the gyro for rotation if available.
            final Pose2d pose = new Pose2d(
                    pose3d.getTranslation().toTranslation2d(),
                    gyroRotation);

            m_log_pose.log(() -> pose);
            m_pub_pose.set(pose);

            if (!Experiments.instance.enabled(Experiment.HeedVision)) {
                // If we've turned vision off altogether, then don't apply this update to the
                // pose estimator.
                if (DEBUG)
                    System.out.println("heedvision is off");
                continue;
            }

            if (blipTransform.getTranslation().getNorm() > m_heedRadiusM) {
                if (DEBUG)
                    System.out.println("tag is too far");
                // Skip too-far tags.
                continue;
            }

            if (m_prevPose == null) {
                // Ignore the very first update since we have no idea if it is far from the
                // previous one.
                if (DEBUG)
                    System.out.println("skip first update");
                m_prevPose = pose;
                continue;
            }

            final double distanceM = distance(m_prevPose, pose);
            if (distanceM > VISION_CHANGE_TOLERANCE_M) {
                // The new estimate is too far from the previous one: it's probably garbage.
                m_prevPose = pose;
                if (DEBUG)
                    System.out.println("too far from previous");
                continue;
            }

            m_visionUpdater.put(
                    correctedTimestamp,
                    pose,
                    stateStdDevs(),
                    visionMeasurementStdDevs(distanceM));

            m_latestTime = Takt.get();
            m_prevPose = pose;
        }
    }

    /** visualization stuff, not used in computation */
    private void extralog(
            double correctedTimestamp,
            Pose2d historicalPose,
            Transform3d cameraOffset,
            Transform3d tagInCamera,
            Pose3d tagInField) {
        // Field-to-robot
        Pose3d historicalPose3d = new Pose3d(historicalPose);
        // Field-to-robot plus robot-to-camera = field-to-camera
        Pose3d historicalCameraInField = historicalPose3d.transformBy(cameraOffset);
        // given the historical pose, where do we think the tag is?
        Pose3d estimatedTagInField = historicalCameraInField.transformBy(tagInCamera);
        m_allTags.add(correctedTimestamp, estimatedTagInField);
        // log the norm of the translational error of the tag.
        Transform3d tagError = tagInField.minus(estimatedTagInField);
        m_log_tag_error.log(() -> tagError.getTranslation().getNorm());
        m_usedTags.add(correctedTimestamp, estimatedTagInField);
    }

    static double[] stateStdDevs() {
        if (Experiments.instance.enabled(Experiment.AvoidVisionJitter)) {
            return tightStateStdDevs;
        }
        return defaultStateStdDevs;
    }

    /** This is an educated guess. */
    static double[] visionMeasurementStdDevs(double distanceM) {
        if (Experiments.instance.enabled(Experiment.AvoidVisionJitter)) {
            /*
             * NEW (3/12/25), 2 cm std dev seems kinda realistic for 1 m.
             * 
             * If it still jitters, try 0.03 or 0.05, but watch out for slow convergence.
             */
            final double K = 0.03;
            return new double[] {
                    (K * distanceM) + 0.01,
                    (K * distanceM) + 0.01,
                    Double.MAX_VALUE };
        }
        /*
         * Standard deviation of pose estimate, as a fraction of target range.
         * This is a guess based on figure 5 in the Apriltag2 paper:
         * https://april.eecs.umich.edu/media/media/pdfs/wang2016iros.pdf
         */
        final double K = 0.03;
        return new double[] {
                K * distanceM,
                K * distanceM,
                Double.MAX_VALUE };
    }

    ///////////////////////////////////////

    /** Distance between pose translations. */
    private static double distance(Pose2d a, Pose2d b) {
        // the translation distance is a little quicker to calculate and we don't care
        // about the "twist" curve measurement
        return GeometryUtil.distance(
                a.getTranslation(),
                b.getTranslation());
    }

}
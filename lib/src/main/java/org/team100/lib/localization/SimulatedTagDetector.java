package org.team100.lib.localization;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Random;
import java.util.function.DoubleFunction;

import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Camera;
import org.team100.lib.state.ModelR3;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Publishes AprilTag Blip24 sightings on Network Tables, just like real
 * cameras would.
 */
public class SimulatedTagDetector {
    private static final boolean DEBUG = false;
    private static final boolean PUBLISH_DEBUG = false;
    // these are the extents of the normalized image coordinates
    // i.e. in WPILib coordinates this would be Y/X and Z/X.
    // our real cameras can see horizontally to about
    // 0.8 on each side. we didn't measure the vertical
    // extent, but it's probably something like 0.6.
    //
    // see
    // https://docs.google.com/spreadsheets/d/1x2_58wyVb5e9HJW8WgakgYcOXgPaJe0yTIHew206M-M
    private static final double HFOV = 0.8;
    private static final double VFOV = 0.6;
    private static final int TAG_COUNT = 22;
    // past about 80 degrees, you can't see the tag.
    private static final double OBLIQUE_LIMIT_RAD = 1.4;
    // camera frame is from 85 ms ago, more or less
    private static final double MEAN_DELAY = 0.085;
    private static final double STDEV_DELAY = 0.02;

    private final List<Camera> m_cameras;
    private final AprilTagFieldLayoutWithCorrectOrientation m_layout;
    private final DoubleFunction<ModelR3> m_history;

    private final Map<Camera, StructArrayPublisher<Blip24>> m_publishers;
    /** client instance, not the default */
    private final NetworkTableInstance m_inst;
    private final Random m_rand;

    /**
     * 
     * @param cameras
     * @param layout
     * @param history pose history by timestamp (sec)
     */
    public SimulatedTagDetector(
            List<Camera> cameras,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            DoubleFunction<ModelR3> history) {
        m_cameras = cameras;
        m_layout = layout;
        m_history = history;
        m_publishers = new HashMap<>();
        m_inst = NetworkTableInstance.create();
        // this is a client just like the camera is a client.
        m_inst.startClient4("SimulatedTagDetector");
        m_inst.setServer("localhost");
        m_rand = new Random();
        for (Camera camera : m_cameras) {
            // see tag_detector.py
            String name = "vision/" + camera.getSerial() + "/0/blips";
            m_publishers.put(
                    camera,
                    m_inst.getStructArrayTopic(
                            name, Blip24.struct).publish());
        }
    }

    public static Runnable get(AprilTagFieldLayoutWithCorrectOrientation layout, SwerveHistory history) {
        if (RobotBase.isReal()) {
            // Real robots get an empty simulated tag detector.
            return () -> {
            };
        } else {
            // In simulation, we want the real simulated tag detector.
            SimulatedTagDetector sim = new SimulatedTagDetector(
                    List.of(
                            Camera.SWERVE_LEFT,
                            Camera.SWERVE_RIGHT,
                            Camera.FUNNEL,
                            Camera.CORAL_LEFT,
                            Camera.CORAL_RIGHT),
                    layout,
                    history);
            return sim::periodic;
        }
    }

    public void periodic() {
        if (DEBUG)
            System.out.println("simulated tag detector");
        Optional<Alliance> opt = DriverStation.getAlliance();
        if (opt.isEmpty())
            return;

        // fetch the pose from a little while ago
        double actualDelay = MEAN_DELAY + m_rand.nextGaussian() * STDEV_DELAY;
        double timestampS = Takt.get() - actualDelay;
        Pose2d pose = m_history.apply(timestampS).pose();

        Pose3d robotPose3d = new Pose3d(pose);
        if (DEBUG) {
            System.out.printf("robot pose X %6.2f Y %6.2f Z %6.2f R %6.2f P %6.2f Y %6.2f \n",
                    robotPose3d.getTranslation().getX(), robotPose3d.getTranslation().getY(),
                    robotPose3d.getTranslation().getZ(), robotPose3d.getRotation().getX(),
                    robotPose3d.getRotation().getY(), robotPose3d.getRotation().getZ());
        }
        for (Map.Entry<Camera, StructArrayPublisher<Blip24>> entry : m_publishers.entrySet()) {
            Camera camera = entry.getKey();
            StructArrayPublisher<Blip24> publisher = entry.getValue();

            List<Blip24> blips = new ArrayList<>();
            Transform3d cameraOffset = camera.getOffset();
            Pose3d cameraPose3d = robotPose3d.plus(cameraOffset);
            Alliance alliance = opt.get();

            for (int tagId = 1; tagId <= TAG_COUNT; ++tagId) {
                if (DEBUG) {
                    System.out.printf("alliance %s camera %12s ", alliance.name(), camera.name());
                }
                Pose3d tagPose = m_layout.getTagPose(alliance, tagId).get();
                if (DEBUG) {
                    System.out.printf("tag id: %2d tag pose: X %6.2f Y %6.2f Z %6.2f R %6.2f P %6.2f Y %6.2f ",
                            tagId, tagPose.getTranslation().getX(), tagPose.getTranslation().getY(),
                            tagPose.getTranslation().getZ(), tagPose.getRotation().getX(), tagPose.getRotation().getY(),
                            tagPose.getRotation().getZ());
                }
                Transform3d tagInCamera = tagInCamera(cameraPose3d, tagPose);
                if (visible(tagInCamera)) {
                    // publish it
                    if (DEBUG) {

                        System.out.print("VISIBLE ");
                    }
                    blips.add(Blip24.fromXForward(tagId, tagInCamera));
                } else {
                    // ignore it
                    if (DEBUG) {

                        System.out.print(" . ");
                    }
                }
                if (DEBUG) {
                    System.out.printf("camera: X %6.2f Y %6.2f Z %6.2f R %6.2f P %6.2f Y %6.2f",
                            cameraOffset.getTranslation().getX(), cameraOffset.getTranslation().getY(),
                            cameraOffset.getTranslation().getZ(), cameraOffset.getRotation().getX(),
                            cameraOffset.getRotation().getY(), cameraOffset.getRotation().getZ());
                    Translation3d tagTranslationInCamera = tagInCamera.getTranslation();
                    Rotation3d tagRotationInCamera = tagInCamera.getRotation();
                    System.out.printf(" tag in camera: X %6.2f Y %6.2f Z %6.2f  R %6.2f P %6.2f Y %6.2f\n",
                            tagTranslationInCamera.getX(), tagTranslationInCamera.getY(),
                            tagTranslationInCamera.getZ(), tagRotationInCamera.getX(), tagRotationInCamera.getY(),
                            tagRotationInCamera.getZ());
                }

            }

            // publish whatever we saw
            // with a timestamp matching the pose above
            long delayUs = (long) actualDelay * 1000000;
            long timestampUs = NetworkTablesJNI.now();
            publisher.set(
                    blips.toArray(new Blip24[0]), timestampUs - delayUs);
            if (PUBLISH_DEBUG) {
                System.out.printf("%s\n", blips);
            }
        }

    }

    /** Return the transform from the camera pose to the tag pose. */
    static Transform3d tagInCamera(Pose3d cameraPose3d, Pose3d tagPose) {
        return new Transform3d(cameraPose3d, tagPose);
    }

    /**
     * If the target is behind the camera, it is never visible.
     */
    static boolean inFront(Transform3d tagInCamera) {
        Translation3d tagTranslationInCamera = tagInCamera.getTranslation();
        double x = tagTranslationInCamera.getX();
        if (x < 0) {
            if (DEBUG) {
                System.out.printf("   behind (%6.2f) ", x);
            }
            return false;
        }
        if (DEBUG) {
            System.out.printf(" in front (%6.2f) ", x);
        }
        return true;
    }

    /**
     * The tag needs to be facing the camera, at least a little.
     * 
     * We compute the angle between the tag normal vector and the translation
     * vector to find the apparent angle.
     */
    static boolean facing(Transform3d tagInCamera) {
        Translation3d tagTranslationInCamera = tagInCamera.getTranslation();
        Rotation3d tagRotationInCamera = tagInCamera.getRotation();
        Translation3d normal = new Translation3d(1, 0, 0);
        // this points "into the page" of the tag
        Translation3d rotatedNormal = normal.rotateBy(tagRotationInCamera);
        Vector<N3> rotatedNormalVector = rotatedNormal.toVector();
        Vector<N3> tagTranslationVector = tagTranslationInCamera.toVector();
        Rotation3d apparentAngle = new Rotation3d(tagTranslationVector, rotatedNormalVector);
        double angle = apparentAngle.getAngle();

        if (Math.abs(angle) > OBLIQUE_LIMIT_RAD) {
            if (DEBUG) {
                System.out.printf(" facing away (%6.2f)", angle);
            }
            return false;
        }
        if (DEBUG) {
            System.out.printf("    angle ok (%6.2f)", angle);
        }
        return true;
    }

    /**
     * The "field of view" is expressed as an angle, but we don't really use an
     * angle, we use the pinhole projection.
     * opencv notation for these normalized coordinates is
     * x'' and y'' so these are x-prime-prime.
     * x is the horizontal dimension, pointing right
     * y is the vertical dimension, pointing down
     * the origin is on the camera bore.
     */
    static boolean inFOV(Transform3d tagInCamera) {
        Translation3d tagTranslationInCamera = tagInCamera.getTranslation();
        double xpp = -1.0 * tagTranslationInCamera.getY() / tagTranslationInCamera.getX();
        double ypp = -1.0 * tagTranslationInCamera.getZ() / tagTranslationInCamera.getX();
        if (Math.abs(xpp) < HFOV && Math.abs(ypp) < VFOV) {
            if (DEBUG) {
                System.out.printf("  FOV IN xpp %6.2f ypp %6.2f ", xpp, ypp);
            }
            return true;
        }
        if (DEBUG) {
            System.out.printf(" FOV OUT xpp %6.2f ypp %6.2f ", xpp, ypp);
        }
        return false;

    }

    static boolean visible(Transform3d tagInCamera) {
        if (!inFront(tagInCamera)) {
            if (DEBUG) {
                System.out.print(" ........................................................");
            }
            return false;
        }

        if (!facing(tagInCamera)) {
            if (DEBUG) {
                System.out.print(" ...................................");
            }
            return false;
        }

        if (!inFOV(tagInCamera)) {
            if (DEBUG) {
                System.out.print(" ... ");
            }
            return false;
        }

        return true;
    }

}

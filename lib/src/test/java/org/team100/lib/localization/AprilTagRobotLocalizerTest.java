package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;

import org.junit.jupiter.api.Test;
import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ModelR3;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;


class AprilTagRobotLocalizerTest implements Timeless {
    private static final double DELTA = 0.01;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testVisionStdDevs() {
        double targetRangeM = 1.0;
        double[] visionStdDev = AprilTagRobotLocalizer.visionMeasurementStdDevs(targetRangeM);
        assertEquals(3, visionStdDev.length);
        assertEquals(0.04, visionStdDev[0], DELTA);
        assertEquals(0.04, visionStdDev[1], DELTA);
        assertEquals(Double.MAX_VALUE, visionStdDev[2], DELTA);
    }

    @Test
    void testStateStdDevs() {
        // these are the "antijitter" values.
        // 1 mm, very low
        double[] stateStdDev = AprilTagRobotLocalizer.tightStateStdDevs;
        assertEquals(3, stateStdDev.length);
        assertEquals(0.001, stateStdDev[0], DELTA);
        assertEquals(0.001, stateStdDev[1], DELTA);
        assertEquals(0.1, stateStdDev[2], DELTA);
    }

    @Test
    void testEndToEnd() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        // these lists receive the updates
        final List<Pose2d> poseEstimate = new ArrayList<Pose2d>();
        final List<Double> timeEstimate = new ArrayList<Double>();
        DoubleFunction<ModelR3> history = t -> new ModelR3();

        VisionUpdater visionUpdater = new VisionUpdater() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                poseEstimate.add(p);
                timeEstimate.add(t);
            }
        };

        AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                logger, layout, history, visionUpdater);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("tag_finder24");
        StructArrayTopic<Blip24> topic = inst.getStructArrayTopic(
                "vision/1234/5678/blips", Blip24.struct);
        StructArrayPublisher<Blip24> pub = topic.publish();
        pub.set(new Blip24[] {
                Blip24.fromXForward(1, new Transform3d(1, 0, 0, new Rotation3d())) },
                (long) Takt.get() * 1000000);

        assertTrue(poseEstimate.isEmpty());
        // localizer needs alliance
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
        localizer.update();
        // skip first update
        assertTrue(poseEstimate.isEmpty());
        // a little bit different so NT will pass it along
        // so about 1 meter ahead; camera offset is identity, but we
        // use gyro rotation, which is zero, so our pose should be 1 meter
        // back along the x axis.
        pub.set(new Blip24[] {
                Blip24.fromXForward(1, new Transform3d(1.01, 0, 0, new Rotation3d())) },
                (long) Takt.get() * 1000000);
        localizer.update();
        assertEquals(1, poseEstimate.size());
        Pose2d pose = poseEstimate.get(0);

        // 1 meter back along x since we override the rotation
        assertEquals(15.697, pose.getX(), DELTA);
        // same y since rotation = 0
        assertEquals(0.665, pose.getY(), DELTA);
        // this is the gyro rotation we told it to use, not the tag rotation.
        assertEquals(0.0, pose.getRotation().getRadians(), DELTA);
    }

    @Test
    void testEstimateRobotPose() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        // these lists receive the updates
        final List<Pose2d> poseEstimate = new ArrayList<Pose2d>();
        final List<Double> timeEstimate = new ArrayList<Double>();

        DoubleFunction<ModelR3> history = t -> new ModelR3();

        VisionUpdater visionUpdater = new VisionUpdater() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                poseEstimate.add(p);
                timeEstimate.add(t);
            }
        };

        AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                logger, layout, history, visionUpdater);

        // in red layout blip 7 is on the other side of the field

        // one meter range (Z forward)
        Blip24 blip = new Blip24(7, new Transform3d(new Translation3d(0, 0, 1), new Rotation3d()));

        // verify tag location
        Pose3d tagPose = layout.getTagPose(Alliance.Red, 7).get();
        assertEquals(3.658, tagPose.getX(), DELTA);
        assertEquals(4.026, tagPose.getY(), DELTA);
        assertEquals(0.308, tagPose.getZ(), DELTA);
        assertEquals(0, tagPose.getRotation().getX(), DELTA);
        assertEquals(0, tagPose.getRotation().getY(), DELTA);
        assertEquals(0, tagPose.getRotation().getZ(), DELTA);

        // final String key = "foo";
        final Blip24[] blips = new Blip24[] {
                blip
        };

        Transform3d cameraOffset = new Transform3d();
        Optional<Alliance> alliance = Optional.of(Alliance.Red);
        localizer.estimateRobotPose(cameraOffset, blips, Takt.get(), alliance);
        // do it twice to convince vdp it's a good estimate
        localizer.estimateRobotPose(cameraOffset, blips, Takt.get(), alliance);
        assertEquals(1, poseEstimate.size());
        assertEquals(1, timeEstimate.size());

        Pose2d result = poseEstimate.get(0);
        assertEquals(2.657, result.getX(), DELTA); // target is one meter in front
        assertEquals(4.026, result.getY(), DELTA); // same y as target
        assertEquals(0, result.getRotation().getRadians(), DELTA); // facing along x
    }

    @Test
    void testEstimateRobotPose2() throws IOException {
        // robot is panned right 45, translation is ignored.
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        final List<Pose2d> poseEstimate = new ArrayList<Pose2d>();
        final List<Double> timeEstimate = new ArrayList<Double>();
        DoubleFunction<ModelR3> history = t -> new ModelR3(new Rotation2d(-Math.PI / 4));
        VisionUpdater visionUpdater = new VisionUpdater() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                poseEstimate.add(p);
                timeEstimate.add(t);
            }
        };

        AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                logger, layout, history, visionUpdater);

        // camera sees the tag straight ahead in the center of the frame,
        // but rotated pi/4 to the left. this is ignored anyway.
        Blip24 blip = new Blip24(7, new Transform3d(
                new Translation3d(0, 0, Math.sqrt(2)),
                new Rotation3d(0, -Math.PI / 4, 0)));

        // verify tag 7 location
        Pose3d tagPose = layout.getTagPose(Alliance.Red, 7).get();
        assertEquals(3.658, tagPose.getX(), DELTA);
        assertEquals(4.026, tagPose.getY(), DELTA);
        assertEquals(0.308, tagPose.getZ(), DELTA);
        assertEquals(0, tagPose.getRotation().getX(), DELTA);
        assertEquals(0, tagPose.getRotation().getY(), DELTA);
        assertEquals(0, tagPose.getRotation().getZ(), DELTA);

        final Blip24[] blips = new Blip24[] { blip };

        Transform3d cameraOffset = new Transform3d();
        Optional<Alliance> alliance = Optional.of(Alliance.Red);
        localizer.estimateRobotPose(cameraOffset, blips, Takt.get() - 0.075, alliance);
        // two good estimates are required, so do another one.
        localizer.estimateRobotPose(cameraOffset, blips, Takt.get() - 0.075, alliance);

        assertEquals(1, poseEstimate.size());
        assertEquals(1, timeEstimate.size());

        Pose2d result = poseEstimate.get(0);
        // robot is is one meter away from the target in x
        assertEquals(2.658, result.getX(), DELTA);
        // robot is one meter to the left (i.e. in y)
        assertEquals(5.026, result.getY(), DELTA);
        // facing diagonal, this is just what we provided.
        assertEquals(-Math.PI / 4, result.getRotation().getRadians(), DELTA);

        // the delay is the input plus the magic number.
        double now = Takt.get();
        Double t = timeEstimate.get(0);
        double delay = now - t;
        assertEquals(0.102, delay, DELTA);
    }

    @Test
    void testRotationInterpolation() {
        // just to be sure of what it's doing
        Rotation2d a = Rotation2d.fromDegrees(10);
        Rotation2d b = Rotation2d.fromDegrees(340);
        Rotation2d c = a.interpolate(b, 0.5);
        assertEquals(-5, c.getDegrees(), DELTA);
    }

    @Test
    void testCase1() throws IOException {

        // the case from 2/14
        // robot 45 degrees to the right (negative), so 135 degrees
        // x = 2.2m, y = - 1.3 m from the center speaker tag
        // camera B
        // camera to tag 4: z=2.4, x=0, y=0 (approx)
        // camera to tag 3: z=2.8, x=0.1, y=0.1 (approx)
        // tag 4 in red is at about (0, 2.5)
        // tag 3 in red is at about (0, 3)

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        DoubleFunction<ModelR3> history = t -> new ModelR3(new Rotation2d(3 * Math.PI / 4));

        VisionUpdater visionUpdater = new VisionUpdater() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                assertEquals(-100, p.getX(), DELTA);
                assertEquals(-100, p.getY(), DELTA);
            }
        };
        AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                logger, layout, history, visionUpdater);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 2.4),
                new Rotation3d()));
        Blip24 tag3 = new Blip24(3, new Transform3d(
                new Translation3d(0.1, 0.1, 2.8),
                new Rotation3d()));

        final Blip24[] tags = new Blip24[] { tag3, tag4 };

        Transform3d cameraOffset = new Transform3d(
                new Translation3d(-0.1265, 0.03, 0.61),
                new Rotation3d(0, Math.toRadians(31.5), Math.PI));
        Optional<Alliance> alliance = Optional.of(Alliance.Red);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);

    }

    @Test
    void testCase2() throws IOException {

        // 1m in front of tag 4
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 2.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);

        DoubleFunction<ModelR3> history = t -> new ModelR3(new Rotation2d(Math.PI));
        VisionUpdater visionUpdater = new VisionUpdater() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                assertEquals(9.272, p.getX(), DELTA);
                assertEquals(1.914, p.getY(), DELTA);
            }
        };
        AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                logger, layout, history, visionUpdater);

        // tag is 1m away on bore
        final Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1),
                new Rotation3d()));

        final Blip24[] tags = new Blip24[] { tag4 };

        Transform3d cameraOffset = new Transform3d();
        Optional<Alliance> alliance = Optional.of(Alliance.Red);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);
    }

    @Test
    void testCase2WithOffset() throws IOException {
        // 1m in front of tag 4
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 2.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);

        DoubleFunction<ModelR3> history = t -> new ModelR3(new Rotation2d(Math.PI));
        VisionUpdater visionUpdater = new VisionUpdater() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                assertEquals(10.272, p.getX(), DELTA);
                assertEquals(1.914, p.getY(), DELTA);
            }
        };

        AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                logger, layout, history, visionUpdater);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1),
                new Rotation3d()));

        final Blip24[] tags = new Blip24[] { tag4 };

        Transform3d cameraOffset = new Transform3d(
                new Translation3d(1, 0, 0),
                new Rotation3d());
        Optional<Alliance> alliance = Optional.of(Alliance.Red);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);
    }

    @Test
    void testCase2WithTriangulation() throws IOException {

        // 1m in front of tag 4
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 2.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);

        DoubleFunction<ModelR3> history = t -> new ModelR3(new Rotation2d(Math.PI));

        VisionUpdater visionUpdater = new VisionUpdater() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                assertEquals(0.96, p.getX(), DELTA);
                assertEquals(2.66, p.getY(), DELTA);
            }
        };

        AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                logger, layout, history, visionUpdater);

        Blip24 tag3 = new Blip24(3, new Transform3d(
                new Translation3d(0.561, 0, 1),
                new Rotation3d()));
        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1),
                new Rotation3d()));

        final Blip24[] tags = new Blip24[] { tag3, tag4 };

        Transform3d cameraOffset = new Transform3d();
        Optional<Alliance> alliance = Optional.of(Alliance.Red);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);
    }

    @Test
    void testCase2tilt() throws IOException {

        // 1m in front of tag 4, tilted up 45
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 2.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);

        DoubleFunction<ModelR3> history = t -> new ModelR3(new Rotation2d(Math.PI));

        VisionUpdater visionUpdater = new VisionUpdater() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                assertEquals(9.272, p.getX(), DELTA);
                assertEquals(1.914, p.getY(), DELTA);
            }
        };

        AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                logger, layout, history, visionUpdater);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1.4142),
                new Rotation3d()));

        final Blip24[] tags = new Blip24[] { tag4 };

        Transform3d cameraOffset = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, Math.PI / 4, 0));
        Optional<Alliance> alliance = Optional.of(Alliance.Red);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);
    }

    @Test
    void testCase3() throws IOException {

        // 1m in front of tag 4, 1m to the right
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);

        DoubleFunction<ModelR3> history = t -> new ModelR3(new Rotation2d(Math.PI));

        VisionUpdater visionUpdater = new VisionUpdater() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                assertEquals(9.272, p.getX(), DELTA);
                assertEquals(2.914, p.getY(), DELTA);
            }
        };

        AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                logger, layout, history, visionUpdater);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(-1, 0, 1),
                new Rotation3d()));

        final Blip24[] tags = new Blip24[] { tag4 };

        Transform3d cameraOffset = new Transform3d();
        Optional<Alliance> alliance = Optional.of(Alliance.Red);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);
    }

    @Test
    void testCase4() throws IOException {

        // 1m in front of tag 4, 1m to the right, rotated to the left
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);

        DoubleFunction<ModelR3> history = t -> new ModelR3(new Rotation2d(-3 * Math.PI / 4));

        VisionUpdater visionUpdater = new VisionUpdater() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                assertEquals(9.272, p.getX(), DELTA);
                assertEquals(2.914, p.getY(), DELTA);
            }
        };

        AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                logger, layout, history, visionUpdater);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1.4142),
                new Rotation3d()));

        final Blip24[] tags = new Blip24[] { tag4 };

        Transform3d cameraOffset = new Transform3d();
        Optional<Alliance> alliance = Optional.of(Alliance.Red);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);
    }

    @Test
    void testCase5() throws IOException {

        // 1m in front of tag 4, 1m to the left, rotated to the right
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);

        DoubleFunction<ModelR3> history = t -> new ModelR3(new Rotation2d(3 * Math.PI / 4));

        VisionUpdater visionUpdater = new VisionUpdater() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                assertEquals(9.272, p.getX(), DELTA);
                assertEquals(0.914, p.getY(), DELTA);
            }
        };
        AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                logger, layout, history, visionUpdater);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1.4142),
                new Rotation3d()));

        final Blip24[] tags = new Blip24[] { tag4 };
        Transform3d cameraOffset = new Transform3d();
        Optional<Alliance> alliance = Optional.of(Alliance.Red);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);
    }

    @Test
    void testCase6() throws IOException {

        // 1m in front of tag 4, 1m to the left, rotated to the right
        // looking down at a 45 degree angle
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);

        DoubleFunction<ModelR3> history = t -> new ModelR3(new Rotation2d(3 * Math.PI / 4));

        VisionUpdater visionUpdater = new VisionUpdater() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                assertEquals(9.272, p.getX(), DELTA);
                assertEquals(0.914, p.getY(), DELTA);
            }
        };
        AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                logger, layout, history, visionUpdater);

        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 2),
                new Rotation3d()));

        final Blip24[] tags = new Blip24[] { tag4 };

        Transform3d cameraOffset = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, Math.PI / 4, 0));
        Optional<Alliance> alliance = Optional.of(Alliance.Red);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);
    }

    @Test
    void testCase7() throws IOException {

        // 1m in front of tag 4, 1m to the left, rotated to the right
        // looking up at a 30 degree angle
        // field is 16.54 m long, 8.21 m wide
        // tag 4 is at 16.579, 5.547, 1.451 in blue so
        // -0.039, 2.662, 1.451 in red.
        // so the robot pose should be 1, 3.662, 1.451

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);

        DoubleFunction<ModelR3> history = t -> new ModelR3(new Rotation2d(3 * Math.PI / 4));

        VisionUpdater visionUpdater = new VisionUpdater() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                assertEquals(9.272, p.getX(), DELTA);
                assertEquals(0.914, p.getY(), DELTA);
            }
        };
        AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                logger, layout, history, visionUpdater);

        // 30 degrees, long side is sqrt2, so hypotenuse is sqrt2/sqrt3/2
        Blip24 tag4 = new Blip24(4, new Transform3d(
                new Translation3d(0, 0, 1.633),
                new Rotation3d()));

        final Blip24[] tags = new Blip24[] { tag4 };

        Transform3d cameraOffset = new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, Math.PI / 6, 0));
        Optional<Alliance> alliance = Optional.of(Alliance.Red);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);
        localizer.estimateRobotPose(cameraOffset, tags, Takt.get(), alliance);
    }
}
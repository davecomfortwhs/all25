
package org.team100.lib.motion.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.Pose2dWithMotion.MotionDirection;
import org.team100.lib.state.ModelR3;
import org.team100.lib.trajectory.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class ModelR3Test {
    private static final double DELTA = 0.001;

    @Test
    void testTransform() {
        Pose2d p = new Pose2d(new Translation2d(1, 1), new Rotation2d(1));
        GlobalVelocityR3 t = new GlobalVelocityR3(1, 1, 1);
        ModelR3 s = new ModelR3(p, t);
        assertEquals(1, s.x().x(), DELTA);
    }

    @Test
    void testTimedPose() {
        ModelR3 s = ModelR3.fromTimedPose(
                new TimedPose(
                        new Pose2dWithMotion(
                                new Pose2d(0, 0, new Rotation2d()),
                                new MotionDirection(0, 0, 0), 0, 0),
                        0, 0, 0));
        assertEquals(0, s.x().x(), DELTA);
        assertEquals(0, s.x().v(), DELTA);
        // assertEquals(0, s.x().a(), DELTA);
        assertEquals(0, s.y().x(), DELTA);
        assertEquals(0, s.y().v(), DELTA);
        // assertEquals(0, s.y().a(), DELTA);
    }

    @Test
    void testTimedPose2() {
        ModelR3 s = ModelR3.fromTimedPose(
                new TimedPose(
                        new Pose2dWithMotion(
                                new Pose2d(0, 0, new Rotation2d()),
                                new MotionDirection(0, 0, 0), 0, 0),
                        0, 0, 1));
        assertEquals(0, s.x().x(), DELTA);
        assertEquals(0, s.x().v(), DELTA);
        // assertEquals(1, s.x().a(), DELTA);
        assertEquals(0, s.y().x(), DELTA);
        assertEquals(0, s.y().v(), DELTA);
        // assertEquals(0, s.y().a(), DELTA);
    }

    @Test
    void testTimedPose3() {
        ModelR3 s = ModelR3.fromTimedPose(
                new TimedPose(
                        new Pose2dWithMotion(
                                new Pose2d(0, 0, new Rotation2d()),
                                new MotionDirection(1, 0, 0), 0, 0),
                        0, 1, 0));
        assertEquals(0, s.x().x(), DELTA);
        assertEquals(1, s.x().v(), DELTA);
        // assertEquals(0, s.x().a(), DELTA);
        assertEquals(0, s.y().x(), DELTA);
        assertEquals(0, s.y().v(), DELTA);
        // assertEquals(0, s.y().a(), DELTA);
    }

    /** +x motion, positive curvature => +y accel. */
    @Test
    void testTimedPose4() {
        ModelR3 s = ModelR3.fromTimedPose(
                new TimedPose(
                        new Pose2dWithMotion(
                                new Pose2d(0, 0, new Rotation2d()),
                                new MotionDirection(1, 0, 0), 1, 0),
                        0, 1, 0));
        assertEquals(0, s.x().x(), DELTA);
        assertEquals(1, s.x().v(), DELTA);
        // assertEquals(0, s.x().a(), DELTA);
        assertEquals(0, s.y().x(), DELTA);
        assertEquals(0, s.y().v(), DELTA);
        // assertEquals(1, s.y().a(), DELTA);
    }

    @Test
    void testChassisSpeeds0() {
        ModelR3 state = new ModelR3(
                new Pose2d(new Translation2d(0, 0), Rotation2d.kPi),
                new GlobalVelocityR3(1, 0, 0));
        ChassisSpeeds speeds = state.chassisSpeeds();
        assertEquals(-1, speeds.vxMetersPerSecond, DELTA);
        assertEquals(0, speeds.vyMetersPerSecond, DELTA);
        assertEquals(0, speeds.omegaRadiansPerSecond, DELTA);
    }

    @Test
    void testChassisSpeeds1() {
        ModelR3 state = new ModelR3(
                new Pose2d(new Translation2d(0, 0), Rotation2d.kCCW_Pi_2),
                new GlobalVelocityR3(1, 0, 1));
        ChassisSpeeds speeds = state.chassisSpeeds();
        assertEquals(0, speeds.vxMetersPerSecond, DELTA);
        assertEquals(-1, speeds.vyMetersPerSecond, DELTA);
        assertEquals(1, speeds.omegaRadiansPerSecond, DELTA);
    }
}

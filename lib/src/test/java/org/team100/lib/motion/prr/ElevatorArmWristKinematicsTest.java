package org.team100.lib.motion.prr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ElevatorArmWristKinematicsTest {
    private static final boolean DEBUG = false;
    // one micrometer tolerance since all the math here is exact
    private static final double DELTA = 0.000001;

    @Test
    void testArmHeightComp() {
        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(5, 1);
        Translation2d wristPosition = new Translation2d(3, 3);
        double h = k.armX(wristPosition);
        if (DEBUG)
            System.out.println(wristPosition.getY());
        assertEquals(4, h);
    }

    @Test
    void testForward0() {
        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(0.3, 0.1);
        Config c = new Config(1, Math.toRadians(60), Math.toRadians(0));
        Pose2d p = k.forward(c);
        // 60 degrees so x is half the total length
        assertEquals(1.2, p.getX(), DELTA);
        // 30/60/90 triangle, this side is sqrt(3)/2
        assertEquals(0.4 * Math.sqrt(3) / 2, p.getY(), DELTA);
        // should be the same as the input
        assertEquals(Math.toRadians(60), p.getRotation().getRadians(), DELTA);
    }

    @Test
    void testForward1() {
        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        // one meter high, zero shoulder (so to the right along x), zero wrist (also
        // along x)
        Config c = new Config(1, 0, 0);
        Pose2d p = k.forward(c);
        // should be the height plus the sum of the link lengths
        assertEquals(4.0, p.getX(), DELTA);
        // straight up
        assertEquals(0.0, p.getY(), DELTA);
        // relative angle should be zero
        assertEquals(Math.toRadians(0), p.getRotation().getRadians(), DELTA);
    }

    @Test
    void testInverse0() {
        // should be straight up
        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        Pose2d p = new Pose2d(4, 0, Rotation2d.kZero);
        Config c = k.inverse(p);
        // pose at 4, total is 3 long, so shoulder at 1
        assertEquals(1, c.shoulderHeight(), DELTA);
        assertEquals(0, c.shoulderAngle(), DELTA);
        assertEquals(0, c.wristAngle(), DELTA);
    }

    @Test
    void testInverseDownArm45Triangle() {
        // built for a 45 45 90 triangle for
        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics((2 * Math.sqrt(2)), 1);
        Pose2d p = new Pose2d(0.1, 3, Rotation2d.kCCW_90deg);
        Config c = k.inverse(p);

        assertEquals(-1.9, c.shoulderHeight(), 0.001);
        assertEquals(Math.toRadians(45), c.shoulderAngle(), 0.001);
        assertEquals(Math.toRadians(45), c.wristAngle(), 0.001);
    }

    @Test
    void testInverseDownArm() {
        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        Pose2d p = new Pose2d(0.1, 2, Rotation2d.kCCW_90deg);
        Config c = k.inverse(p);

        assertEquals(0.1 - Math.sqrt(3), c.shoulderHeight(), 0.001);
        assertEquals(Math.toRadians(30), c.shoulderAngle(), 0.001);
        assertEquals(Math.toRadians(60), c.wristAngle(), 0.001);
    }

    @Test
    void testInverse1() {
        // arm up, wrist to the side
        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        Pose2d p = new Pose2d(3, 1, Rotation2d.kCCW_90deg);
        Config c = k.inverse(p);
        // arm length is 2, wrist location is at 3
        assertEquals(1, c.shoulderHeight(), DELTA);
        assertEquals(0, c.shoulderAngle(), DELTA);
        assertEquals(Math.PI / 2, c.wristAngle(), DELTA);
    }

    @Test
    void testInverse2() {
        // arm to the side, wrist down
        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        Pose2d p = new Pose2d(0, 2, Rotation2d.k180deg);
        Config c = k.inverse(p);
        assertEquals(1, c.shoulderHeight(), DELTA);
        assertEquals(Math.PI / 2, c.shoulderAngle(), DELTA);
        assertEquals(Math.PI / 2, c.wristAngle(), DELTA);
    }

    @Test
    void testRoundTripInverseFirst() {
        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(0.3, 0.1);
        Pose2d p = new Pose2d(1.178, 0.207, new Rotation2d(Math.toRadians(55)));

        Config c2 = k.inverse(p);
        Pose2d p2 = k.forward(c2);

        assertEquals(p.getX(), p2.getX(), DELTA);
        assertEquals(p.getY(), p2.getY(), DELTA);
        assertEquals(p.getRotation().getRadians(), p2.getRotation().getRadians(), DELTA);

    }

    @Test
    void testRoundTripForwardFirst() {
        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(0.3, 0.1);
        Config c = new Config(1, Math.toRadians(60), Math.toRadians(60));

        Pose2d p2 = k.forward(c);
        assertEquals(1.1, p2.getX(), DELTA);
        assertEquals(0.3 * Math.sqrt(3) / 2 + 0.1 * Math.sqrt(3) / 2, p2.getY(), DELTA);
        assertEquals(Math.toRadians(120), p2.getRotation().getRadians(), DELTA);

        Config c2 = k.inverse(p2);
        assertEquals(c.shoulderHeight(), c2.shoulderHeight(), DELTA);
        assertEquals(c.shoulderAngle(), c2.shoulderAngle(), DELTA);
        assertEquals(c.wristAngle(), c2.wristAngle(), DELTA);
    }

}

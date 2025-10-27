package org.team100.lib.motion.swerve.module;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModule100Test {
    private static final double DELTA = 0.001;

    @Test
    void testCrossTrackError1() {
        double measurement = 0;
        double desiredSpeed = 1;
        Rotation2d desiredAngle = new Rotation2d(0);
        double speed = SwerveModule100.reduceCrossTrackError(
                measurement, desiredSpeed, desiredAngle);
        assertEquals(1, speed, DELTA);
    }

    @Test
    void testCrossTrackError2() {
        double measurement = 0;
        double desiredSpeed = 1;
        Rotation2d desiredAngle = new Rotation2d(Math.PI / 3);
        double speed = SwerveModule100.reduceCrossTrackError(
                measurement, desiredSpeed, desiredAngle);
        // this is the value for cosine
        // assertEquals(0.5, speed, DELTA);
        // this is the value for gaussian
        assertEquals(0.012, speed, DELTA);
    }

    @Test
    void testCrossTrackError3() {
        double measurement = 0;
        double desiredSpeed = 1;
        Rotation2d desiredAngle = new Rotation2d(Math.PI / 6);
        double speed = SwerveModule100.reduceCrossTrackError(
                measurement, desiredSpeed, desiredAngle);
        // this is the value for cosine
        // assertEquals(0.866, speed, DELTA);
        // this is the value for gaussian
        assertEquals(0.334, speed, DELTA);
    }
}

package org.team100.lib.motion.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.swerve.module.state.SwerveModuleDelta;
import org.team100.lib.motion.swerve.module.state.SwerveModulePosition100;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleDeltaTest {
    private static final double DELTA = 0.001;

    @Test
    void testOneModule() {
        {
            SwerveModulePosition100 start = new SwerveModulePosition100(
                    0, Optional.of(Rotation2d.kZero));
            SwerveModulePosition100 end = new SwerveModulePosition100(
                    0, Optional.of(Rotation2d.kZero));
            SwerveModuleDelta delta = SwerveModuleDelta.delta(start, end);
            assertEquals(0, delta.distanceMeters, DELTA);
            assertEquals(0, delta.wrappedAngle.get().getRadians(), DELTA);
        }
        {
            SwerveModulePosition100 start = new SwerveModulePosition100(
                    0, Optional.of(Rotation2d.kZero));
            SwerveModulePosition100 end = new SwerveModulePosition100(
                    1, Optional.of(Rotation2d.kZero));
            SwerveModuleDelta delta = SwerveModuleDelta.delta(start, end);
            assertEquals(1, delta.distanceMeters, DELTA);
            assertEquals(0, delta.wrappedAngle.get().getRadians(), DELTA);
        }
        {
            // this ignores the initial zero rotation, and acts as if
            // the path is at 90 degrees the whole time.
            SwerveModulePosition100 start = new SwerveModulePosition100(
                    0, Optional.of(Rotation2d.kZero));
            SwerveModulePosition100 end = new SwerveModulePosition100(
                    1, Optional.of(Rotation2d.kCCW_Pi_2));
            SwerveModuleDelta delta = SwerveModuleDelta.delta(start, end);
            assertEquals(1, delta.distanceMeters, DELTA);
            assertEquals(1.571, delta.wrappedAngle.get().getRadians(), DELTA);
        }
    }
}

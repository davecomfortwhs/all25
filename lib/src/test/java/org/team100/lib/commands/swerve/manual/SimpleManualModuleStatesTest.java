package org.team100.lib.commands.swerve.manual;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.swerve.module.state.SwerveModuleStates;


class SimpleManualModuleStatesTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testZero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        SimpleManualModuleStates s = new SimpleManualModuleStates(logger, limits);
        Velocity input = new Velocity(0, 0, 0);
        SwerveModuleStates ms = s.apply(input);
        assertEquals(0, ms.frontLeft().angle().get().getRadians(), DELTA);
        assertEquals(0, ms.frontRight().angle().get().getRadians(), DELTA);
        assertEquals(0, ms.rearLeft().angle().get().getRadians(), DELTA);
        assertEquals(0, ms.rearRight().angle().get().getRadians(), DELTA);

        assertEquals(0, ms.frontLeft().speedMetersPerSecond(), DELTA);
        assertEquals(0, ms.frontRight().speedMetersPerSecond(), DELTA);
        assertEquals(0, ms.rearLeft().speedMetersPerSecond(), DELTA);
        assertEquals(0, ms.rearRight().speedMetersPerSecond(), DELTA);
    }

    @Test
    void testAngle() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        SimpleManualModuleStates s = new SimpleManualModuleStates(logger, limits);
        Velocity input = new Velocity(0, 0, 0.5);
        SwerveModuleStates ms = s.apply(input);
        assertEquals(Math.PI / 2, ms.frontLeft().angle().get().getRadians(), DELTA);
        assertEquals(Math.PI / 2, ms.frontRight().angle().get().getRadians(), DELTA);
        assertEquals(Math.PI / 2, ms.rearLeft().angle().get().getRadians(), DELTA);
        assertEquals(Math.PI / 2, ms.rearRight().angle().get().getRadians(), DELTA);

        assertEquals(0, ms.frontLeft().speedMetersPerSecond(), DELTA);
        assertEquals(0, ms.frontRight().speedMetersPerSecond(), DELTA);
        assertEquals(0, ms.rearLeft().speedMetersPerSecond(), DELTA);
        assertEquals(0, ms.rearRight().speedMetersPerSecond(), DELTA);
    }

    @Test
    void testDrive() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        SimpleManualModuleStates s = new SimpleManualModuleStates(logger, limits);
        Velocity input = new Velocity(0.5, 0, 0);
        SwerveModuleStates ms = s.apply(input);
        assertEquals(0, ms.frontLeft().angle().get().getRadians(), DELTA);
        assertEquals(0, ms.frontRight().angle().get().getRadians(), DELTA);
        assertEquals(0, ms.rearLeft().angle().get().getRadians(), DELTA);
        assertEquals(0, ms.rearRight().angle().get().getRadians(), DELTA);

        assertEquals(0.5, ms.frontLeft().speedMetersPerSecond(), DELTA);
        assertEquals(0.5, ms.frontRight().speedMetersPerSecond(), DELTA);
        assertEquals(0.5, ms.rearLeft().speedMetersPerSecond(), DELTA);
        assertEquals(0.5, ms.rearRight().speedMetersPerSecond(), DELTA);
    }

}

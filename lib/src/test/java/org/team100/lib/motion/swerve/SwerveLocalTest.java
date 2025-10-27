package org.team100.lib.motion.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.swerve.module.state.SwerveModuleState100;
import org.team100.lib.motion.swerve.module.state.SwerveModuleStates;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class SwerveLocalTest extends Fixtured implements Timeless {
    public SwerveLocalTest() throws IOException {
    }

    private static final double DELTA = 0.001;

    @Test
    void testSimple() {
        SwerveLocal local = fixture.swerveLocal;
        local.setChassisSpeeds(new ChassisSpeeds());
        local.defense();
        local.stop();
        local.setRawModuleStates(new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d()))));
        assertEquals(0, local.positions().frontLeft().distanceMeters, DELTA);
    }
}

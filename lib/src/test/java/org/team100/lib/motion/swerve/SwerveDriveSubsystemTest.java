package org.team100.lib.motion.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;

import org.junit.jupiter.api.Test;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class SwerveDriveSubsystemTest extends Fixtured implements Timeless {
    public SwerveDriveSubsystemTest() throws IOException {
    }

    private static final double DELTA = 0.01;

    @Test
    void testWithSetpointGenerator() {
        Experiments.instance.testOverride(Experiment.UseSetpointGenerator, true);

        SwerveDriveSubsystem drive = fixture.drive;

        drive.resetPose(new Pose2d());

        stepTime();
        drive.periodic();
        verify(drive, 0, 0, 0);

        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0));

        // actuation is reflected in measurement after time passes
        assertEquals(0, fixture.collection.states().frontLeft().speedMetersPerSecond());
        stepTime();
        assertEquals(1, fixture.collection.states().frontLeft().speedMetersPerSecond());

        drive.periodic();
        assertEquals(0.02, fixture.collection.positions().frontLeft().distanceMeters, 1e-6);

        assertEquals(1, fixture.collection.states().frontLeft().speedMetersPerSecond());

        // the acceleration limit is applied here
        verify(drive, 0.02, 1, 1.0);

        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0));

        stepTime();
        drive.periodic();

        verify(drive, 0.039, 1, 1.0);

        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0));

        stepTime();
        drive.periodic();

        verify(drive, 0.06, 1, 0.06);

        drive.close();
    }

    @Test
    void testWithoutSetpointGenerator() {
        Experiments.instance.testOverride(Experiment.UseSetpointGenerator, false);
        SwerveDriveSubsystem drive = fixture.drive;
        fixture.collection.reset();
        stepTime();

        drive.resetPose(new Pose2d());

        stepTime();
        drive.periodic();

        verify(drive, 0, 0, 0);

        // go 1 m/s in +x
        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0));

        stepTime();
        drive.periodic();

        // at 1 m/s for 0.02 s, so we go 0.02 m
        assertEquals(0.02, fixture.collection.positions().frontLeft().distanceMeters, 1e-6);

        // it took 0.02 s to go from 0 m/s to 1 m/s, so we accelerated 50 m/s/s.
        verify(drive, 0.02, 1.00, 50.0);
        
        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0));

        stepTime();
        drive.periodic();

        // we went a little further, no longer accelerating.
        verify(drive, 0.04, 1.00, 0.0);

        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0));

        stepTime();
        drive.periodic();

        // a little further, but no longer accelerating
        verify(drive, 0.06, 1.00, 0.0);

        drive.close();
    }

    private void verify(SwerveDriveSubsystem drive, double x, double v, double a) {
        assertEquals(x, drive.getPose().getX(), DELTA);
        assertEquals(v, drive.getVelocity().x(), DELTA);
        // assertEquals(a, drive.getState().acceleration().x(), DELTA);
        assertEquals(x, drive.getState().x().x(), DELTA);
        assertEquals(v, drive.getState().x().v(), DELTA);
        // assertEquals(a, drive.getState().x().a(), DELTA);
    }
}

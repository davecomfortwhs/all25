package org.team100.lib.commands.swerve.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.swerve.Fixtured;
import org.team100.lib.motion.swerve.SwerveDriveSubsystem;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.wpilibj.RobotController;

class DriveManuallyTest extends Fixtured implements Timeless {
    public DriveManuallyTest() throws IOException {
    }

    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    String desiredMode = null;
    Velocity desiredTwist = new Velocity(1, 0, 0);

    @Test
    void testSimple() {
        Supplier<Velocity> twistSupplier = () -> desiredTwist;
        SwerveDriveSubsystem drive = fixture.drive;
        fixture.collection.reset();
        stepTime();
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        SwerveLimiter limiter = new SwerveLimiter(
                logger,
                swerveKinodynamics,
                RobotController::getBatteryVoltage);
        DriveManually command = new DriveManually(
                twistSupplier,
                (x) -> {
                },
                drive,
                limiter);

        command.register("MODULE_STATE", false,
                new SimpleManualModuleStates(logger, swerveKinodynamics));

        command.register("ROBOT_RELATIVE_CHASSIS_SPEED", false,
                new ManualChassisSpeeds(logger, swerveKinodynamics));

        command.register("FIELD_RELATIVE_TWIST", false,
                new ManualFieldRelativeSpeeds(logger, swerveKinodynamics));

        command.overrideMode(() -> desiredMode);

        command.initialize();

        desiredMode = "MODULE_STATE";
        command.execute();

        stepTime();
        drive.periodic();
        stepTime();
        assertEquals(1, drive.getChassisSpeeds().vxMetersPerSecond, 0.001);

        desiredMode = "ROBOT_RELATIVE_CHASSIS_SPEED";
        command.execute();

        desiredMode = "FIELD_RELATIVE_TWIST";
        command.execute();

        command.end(false);
    }

}

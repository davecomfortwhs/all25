package org.team100.lib.motion.swerve;

import java.io.IOException;

import org.team100.lib.controller.r3.ControllerFactoryR3;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.gyro.Gyro;
import org.team100.lib.gyro.SimulatedGyro;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.AprilTagRobotLocalizer;
import org.team100.lib.localization.FreshSwerveEstimate;
import org.team100.lib.localization.NudgingVisionUpdater;
import org.team100.lib.localization.OdometryUpdater;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.motion.swerve.module.SwerveModuleCollection;
import org.team100.lib.motion.swerve.module.state.SwerveModulePositions;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A real swerve subsystem populated with simulated motors and encoders,
 * for testing.
 * 
 * Uses simulated position sensors, must be used with clock control (e.g.
 * {@link Timeless}).
 */
public class RealisticFixture {
    public SwerveModuleCollection collection;
    public Gyro gyro;
    public SwerveHistory history;
    public FreshSwerveEstimate estimate;
    public SwerveKinodynamics swerveKinodynamics;
    public SwerveLocal swerveLocal;
    public SwerveDriveSubsystem drive;
    public ControllerR3 controller;
    public LoggerFactory logger;
    public LoggerFactory fieldLogger;

    public RealisticFixture() throws IOException {
        logger = new TestLoggerFactory(new TestPrimitiveLogger());
        fieldLogger = new TestLoggerFactory(new TestPrimitiveLogger());
        swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        collection = SwerveModuleCollection.get(logger, 10, 20, swerveKinodynamics);
        gyro = new SimulatedGyro(logger, swerveKinodynamics, collection);
        swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);
        history = new SwerveHistory(
                swerveKinodynamics,
                Rotation2d.kZero,
                SwerveModulePositions.kZero(),
                Pose2d.kZero,
                0);
        OdometryUpdater odometryUpdater = new OdometryUpdater(swerveKinodynamics, gyro, history, collection::positions);
        odometryUpdater.reset(Pose2d.kZero, 0);

        final NudgingVisionUpdater visionUpdater = new NudgingVisionUpdater(history, odometryUpdater);

        final AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                logger, layout, history, visionUpdater);
        estimate = new FreshSwerveEstimate(localizer, odometryUpdater, history);

        SwerveLimiter limiter = new SwerveLimiter(logger, swerveKinodynamics, () -> 12);

        drive = new SwerveDriveSubsystem(
                fieldLogger,
                logger,
                swerveKinodynamics,
                odometryUpdater,
                estimate,
                swerveLocal,
                limiter);

        controller = ControllerFactoryR3.test(logger);
    }

    public void close() {
        // close the DIO inside the turning encoder
        collection.close();
    }

}

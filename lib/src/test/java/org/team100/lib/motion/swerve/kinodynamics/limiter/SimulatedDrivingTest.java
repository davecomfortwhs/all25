package org.team100.lib.motion.swerve.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;

import org.junit.jupiter.api.Test;
import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.GlobalVelocityR3;
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
import org.team100.lib.motion.swerve.SwerveDriveSubsystem;
import org.team100.lib.motion.swerve.SwerveLocal;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.swerve.module.SwerveModuleCollection;
import org.team100.lib.motion.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.motion.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.motion.swerve.module.state.SwerveModulePositions;
import org.team100.lib.motion.swerve.module.state.SwerveModuleStates;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SimulatedDrivingTest implements Timeless {
    private static final boolean DEBUG = false;
    final LoggerFactory fieldLogger = new TestLoggerFactory(new TestPrimitiveLogger());
    LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
    SwerveModuleCollection collection = SwerveModuleCollection.forTest(
            logger, swerveKinodynamics);

    final Gyro gyro;
    final SwerveHistory history;
    final SwerveLocal swerveLocal;
    final OdometryUpdater odometryUpdater;
    final SwerveLimiter limiter;
    final SwerveDriveSubsystem drive;

    SimulatedDrivingTest() throws IOException {
        gyro = new SimulatedGyro(logger, swerveKinodynamics, collection);
        swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);
        history = new SwerveHistory(
                swerveKinodynamics,
                Rotation2d.kZero,
                SwerveModulePositions.kZero(),
                Pose2d.kZero,
                0);
        odometryUpdater = new OdometryUpdater(swerveKinodynamics, gyro, history, collection::positions);
        odometryUpdater.reset(Pose2d.kZero, 0);

        NudgingVisionUpdater visionUpdater = new NudgingVisionUpdater(history, odometryUpdater);
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                logger, layout, history, visionUpdater);

        FreshSwerveEstimate estimate = new FreshSwerveEstimate(
                localizer, odometryUpdater, history);
        limiter = new SwerveLimiter(logger, swerveKinodynamics, () -> 12);

        drive = new SwerveDriveSubsystem(
                fieldLogger,
                logger,
                swerveKinodynamics,
                odometryUpdater,
                estimate,
                swerveLocal,
                limiter);
    }

    @Test
    void testSteps() {
        GlobalVelocityR3 input = new GlobalVelocityR3(2, 0, 3.5);
        Rotation2d theta = new Rotation2d();
        ChassisSpeeds targetChassisSpeeds = SwerveKinodynamics.toInstantaneousChassisSpeeds(input, theta);
        SwerveModuleStates states = swerveKinodynamics.toSwerveModuleStates(targetChassisSpeeds);

        // mmmm the angles start as zero? does this matter? no?
        SwerveModulePositions startPositions = new SwerveModulePositions(
                new SwerveModulePosition100(),
                new SwerveModulePosition100(),
                new SwerveModulePosition100(),
                new SwerveModulePosition100());

        // say each module proceeds at its setpoint speed and angle (i.e. starting angle
        // is irrelevant)

        double dt = 0.02;
        SwerveModulePositions endPositions = new SwerveModulePositions(
                new SwerveModulePosition100(
                        states.frontLeft().speedMetersPerSecond() * dt,
                        states.frontLeft().angle()),
                new SwerveModulePosition100(
                        states.frontRight().speedMetersPerSecond() * dt,
                        states.frontRight().angle()),
                new SwerveModulePosition100(
                        states.rearLeft().speedMetersPerSecond() * dt,
                        states.rearLeft().angle()),
                new SwerveModulePosition100(
                        states.rearRight().speedMetersPerSecond() * dt,
                        states.rearRight().angle()));

        SwerveModuleDeltas modulePositionDelta = SwerveModuleDeltas.modulePositionDelta(
                startPositions,
                endPositions);
        if (DEBUG)
            System.out.printf("%s\n", modulePositionDelta);

        Twist2d twist = swerveKinodynamics.getKinematics().toTwist2d(modulePositionDelta);

        Pose2d deltaPose = GeometryUtil.sexp(twist);
        ChassisSpeeds continuousSpeeds = new ChassisSpeeds(
                deltaPose.getX(),
                deltaPose.getY(),
                deltaPose.getRotation().getRadians()).div(dt);

        // to pass, this requires the "veering correction" to be zero.
        assertEquals(0, continuousSpeeds.vyMetersPerSecond, 1e-12);
    }

    @Test
    void testStraight() {
        // just +x
        collection.reset();
        GlobalVelocityR3 input = new GlobalVelocityR3(2, 0, 0);
        double start = Takt.get();
        for (int i = 0; i < 100; ++i) {
            stepTime();
            drive.setVelocity(input);
            if (DEBUG)
                System.out.printf("%.2f %s\n", Takt.get() - start, drive.getPose());
        }
    }

    @Test
    void testStraightVerbatim() {
        // just +x
        // this accelerates infinitely, immediately to the requested speed.
        collection.reset();
        GlobalVelocityR3 input = new GlobalVelocityR3(2, 0, 0);
        double start = Takt.get();
        for (int i = 0; i < 100; ++i) {
            stepTime();
            drive.setVelocity(input);
            if (DEBUG)
                System.out.printf("%.2f %s\n", Takt.get() - start, drive.getPose());
        }
    }

    /**
     * Uses the setpoint generator. turn on DEBUG in SwerveLocal to see the bug, the
     * setpoint generator output is not course-invariant.
     * 
     * accel is 10 m/s/s; dt is 0.02, so dv is 0.2.
     */
    @Test
    void testVeering() {
        Experiments.instance.testOverride(Experiment.UseSetpointGenerator, true);
        collection.reset();
        // +x and spinning. course is always zero.
        GlobalVelocityR3 input = new GlobalVelocityR3(2, 0, 3.5);
        for (int i = 0; i < 50; ++i) {
            if (DEBUG)
                System.out.printf("\nstep time ...\n");
            stepTime();
            if (DEBUG)
                System.out.printf("takt: %.2f state: %s\n", Takt.get(), drive.getState());
            drive.setVelocity(input);
        }
    }

    /**
     * No veering. Drive commands go to simulated motors, which respond instantly.
     */
    @Test
    void testVeeringVerbatim() {
        collection.reset();
        // +x and spinning
        GlobalVelocityR3 input = new GlobalVelocityR3(2, 0, 3.5);
        for (int i = 0; i < 100; ++i) {
            if (DEBUG)
                System.out.printf("\nstep time ...\n");
            stepTime();
            if (DEBUG)
                System.out.printf("takt: %.2f state: %s\n", Takt.get(), drive.getState());
            drive.setVelocity(input);
        }
    }

    /** Is the gyro in sync with the estimated pose? Yes. */
    @Test
    void testGyro() {
        // spin fast
        GlobalVelocityR3 input = new GlobalVelocityR3(0, 0, 4);
        if (DEBUG)
            System.out.printf("pose %s, gyro %s, rate %f\n",
                    drive.getPose(),
                    gyro.getYawNWU(),
                    gyro.getYawRateNWU());
        drive.setVelocity(input);
        if (DEBUG)
            System.out.printf("pose %s, gyro %s, rate %f\n",
                    drive.getPose(),
                    gyro.getYawNWU(),
                    gyro.getYawRateNWU());
        stepTime();
        if (DEBUG)
            System.out.printf("pose %s, gyro %s, rate %f\n",
                    drive.getPose(),
                    gyro.getYawNWU(),
                    gyro.getYawRateNWU());
        drive.setVelocity(input);
        if (DEBUG)
            System.out.printf("pose %s, gyro %s, rate %f\n",
                    drive.getPose(),
                    gyro.getYawNWU(),
                    gyro.getYawRateNWU());
        stepTime();
        if (DEBUG)
            System.out.printf("pose %s, gyro %s, rate %f\n",
                    drive.getPose(),
                    gyro.getYawNWU(),
                    gyro.getYawRateNWU());

    }
}

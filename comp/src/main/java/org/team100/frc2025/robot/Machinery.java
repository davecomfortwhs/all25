package org.team100.frc2025.robot;

import java.io.IOException;

import org.team100.frc2025.CalgamesArm.CalgamesMech;
import org.team100.frc2025.CalgamesArm.CalgamesViz;
import org.team100.frc2025.Climber.Climber;
import org.team100.frc2025.Climber.ClimberIntake;
import org.team100.frc2025.Climber.ClimberVisualization;
import org.team100.frc2025.grip.Manipulator;
import org.team100.frc2025.indicator.LEDIndicator;
import org.team100.lib.coherence.Takt;
import org.team100.lib.gyro.Gyro;
import org.team100.lib.gyro.GyroFactory;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.AprilTagRobotLocalizer;
import org.team100.lib.localization.NudgingVisionUpdater;
import org.team100.lib.localization.OdometryUpdater;
import org.team100.lib.localization.SimulatedTagDetector;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.motion.swerve.SwerveDriveFactory;
import org.team100.lib.motion.swerve.SwerveDriveSubsystem;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.swerve.module.SwerveModuleCollection;
import org.team100.lib.targeting.SimulatedTargetWriter;
import org.team100.lib.targeting.Targets;
import org.team100.lib.util.CanId;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * This should contain all the hardware of the robot: all the subsystems etc
 * that the Binder and Auton classes may want to use.
 */
public class Machinery {
    // for background on drive current limits:
    // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
    // https://www.chiefdelphi.com/t/the-brushless-era-needs-sensible-default-current-limits/461056/51
    // https://docs.google.com/document/d/10uXdmu62AFxyolmwtDY8_9UNnci7eVcev4Y64ZS0Aqk
    // https://github.com/frc1678/C2024-Public/blob/17e78272e65a6ce4f87c00a3514c79f787439ca1/src/main/java/com/team1678/frc2024/Constants.java#L195
    // 2/26/25: Joel updated the supply limit to 90A, see 1678 code above. This is
    // essentially unlimited, so you'll need to run some other kind of limiter (e.g.
    // acceleration) to keep from browning out.
    private static final double DRIVE_SUPPLY_LIMIT = 90;
    private static final double DRIVE_STATOR_LIMIT = 110;

    private final Runnable m_combinedViz;
    private final Runnable m_climberViz;
    private final SwerveModuleCollection m_modules;
    private final Runnable m_simulatedTagDetector;
    private final Runnable m_targetSimulator;
    private final LEDIndicator m_leds;

    final FieldLogger.Log m_fieldLog;
    final CalgamesMech m_mech;
    final Manipulator m_manipulator;
    final Climber m_climber;
    final ClimberIntake m_climberIntake;
    final TrajectoryVisualization m_trajectoryViz;
    final SwerveKinodynamics m_swerveKinodynamics;
    final AprilTagRobotLocalizer m_localizer;
    final Targets m_targets;
    final SwerveDriveSubsystem m_drive;
    final Beeper m_beeper;

    public Machinery() {
        final LoggerFactory logger = Logging.instance().rootLogger;
        final LoggerFactory fieldLogger = Logging.instance().fieldLogger;
        final LoggerFactory driveLog = logger.name("Drive");

        m_fieldLog = new FieldLogger.Log(fieldLogger);

        m_swerveKinodynamics = SwerveKinodynamicsFactory.get();

        ////////////////////////////////////////////////////////////
        //
        // SUBSYSTEMS
        //
        m_mech = new CalgamesMech(logger, 0.5, 0.343);
        m_manipulator = new Manipulator(logger);
        m_climber = new Climber(logger, new CanId(13));
        m_climberIntake = new ClimberIntake(logger, new CanId(14));

        ////////////////////////////////////////////////////////////
        //
        // VISUALIZATIONS
        //
        m_trajectoryViz = new TrajectoryVisualization(fieldLogger);
        m_combinedViz = new CalgamesViz(m_mech);
        m_climberViz = new ClimberVisualization(m_climber, m_climberIntake);

        ////////////////////////////////////////////////////////////
        //
        // POSE ESTIMATION
        //
        m_modules = SwerveModuleCollection.get(
                driveLog,
                DRIVE_SUPPLY_LIMIT,
                DRIVE_STATOR_LIMIT,
                m_swerveKinodynamics);
        final Gyro gyro = GyroFactory.get(
                driveLog,
                m_swerveKinodynamics,
                m_modules);
        final SwerveHistory history = new SwerveHistory(
                m_swerveKinodynamics,
                gyro.getYawNWU(),
                m_modules.positions(),
                Pose2d.kZero,
                Takt.get());
        final OdometryUpdater odometryUpdater = new OdometryUpdater(
                m_swerveKinodynamics, gyro, history, m_modules::positions);
        odometryUpdater.reset(Pose2d.kZero);
        final NudgingVisionUpdater visionUpdater = new NudgingVisionUpdater(
                history, odometryUpdater);

        ////////////////////////////////////////////////////////////
        //
        // CAMERA READERS
        //
        final AprilTagFieldLayoutWithCorrectOrientation layout = getLayout();

        m_localizer = new AprilTagRobotLocalizer(
                driveLog,
                layout,
                history,
                visionUpdater);
        m_targets = new Targets(driveLog, m_fieldLog, history);

        ////////////////////////////////////////////////////////////
        //
        // SIMULATED CAMERAS
        //
        m_simulatedTagDetector = SimulatedTagDetector.get(layout, history);
        m_targetSimulator = SimulatedTargetWriter.get(history);

        ////////////////////////////////////////////////////////////
        //
        // DRIVETRAIN
        //
        m_drive = SwerveDriveFactory.get(
                fieldLogger,
                driveLog,
                m_swerveKinodynamics,
                m_localizer,
                odometryUpdater,
                history,
                m_modules);
        m_drive.resetPose(new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d(Math.PI)));

        ////////////////////////////////////////////////////////////
        //
        // LED INDICATOR
        //
        m_leds = new LEDIndicator(
                m_localizer,
                m_manipulator,
                m_climberIntake);
        m_beeper = new Beeper(this);
    }

    public void periodic() {
        // publish the simulated tag sightings.
        m_simulatedTagDetector.run();
        // publish simulated target sightings
        m_targetSimulator.run();
        // show the closest target on field2d
        m_targets.periodic();
        m_leds.periodic();
        m_combinedViz.run();
        m_climberViz.run();
    }

    public void close() {
        // this keeps the tests from conflicting via the use of simulated HAL ports.
        m_modules.close();
        m_leds.close();
    }

    /** Trap the IO exception. */
    private static AprilTagFieldLayoutWithCorrectOrientation getLayout() {
        try {
            return new AprilTagFieldLayoutWithCorrectOrientation();
        } catch (IOException e) {
            throw new IllegalStateException("Could not read Apriltag layout file", e);
        }
    }
}

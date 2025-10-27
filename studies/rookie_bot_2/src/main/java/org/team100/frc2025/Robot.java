package org.team100.frc2025;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.commands.mecanum.ManualMecanum;
import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.indicator.Alerts;
import org.team100.lib.indicator.SolidIndicator;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.motion.mecanum.MecanumDrive100;
import org.team100.lib.motion.mecanum.MecanumDriveFactory;
import org.team100.lib.motion.mecanum.MecanumKinematics100.Slip;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.util.Banner;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Robot extends TimedRobot100 {
    private static final double MAX_SPEED_X_M_S = 3.5;
    private static final double MAX_SPEED_Y_M_S = 3.5;
    private static final double MAX_OMEGA_RAD_S = 5.0;

    private final MecanumDrive100 m_drive;
    private final Autons m_autons;
    private final SolidIndicator m_indicator;
    private final Alerts m_alerts;
    private final Alert m_noStartingPosition;
    private final Alert m_mismatchedAlliance;

    public Robot() {
        Banner.printBanner();
        DriverStation.silenceJoystickConnectionWarning(true);
        Experiments.instance.show();
        SmartDashboard.putData(CommandScheduler.getInstance());
        m_indicator = new SolidIndicator(new RoboRioChannel(0), 40);
        m_alerts = new Alerts();
        m_noStartingPosition = m_alerts.add("No starting position!", AlertType.kWarning);
        m_mismatchedAlliance = m_alerts.add("Wrong Alliance!", AlertType.kWarning);
        m_indicator.state(() -> m_alerts.any() ? Color.kRed : Color.kGreen);
        Logging logging = Logging.instance();
        LoggerFactory fieldLogger = logging.fieldLogger;
        DriverXboxControl driverControl = new DriverXboxControl(0);
        LoggerFactory logger = logging.rootLogger;
        m_drive = MecanumDriveFactory.make(
                fieldLogger,
                logger,
                25, // stator current limit (a)
                new CanId(60), // gyro
                new CanId(2), // front left
                new CanId(1), // front right
                new CanId(3), // rear left
                new CanId(4), // rear right
                0.533, // track width (m)
                0.406, // wheelbase (m)
                new Slip(1, 1, 1), // wheel slip corrections
                6.0, // gears
                0.15); // wheel dia (m)
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.mecanum();
        SwerveLimiter limiter = new SwerveLimiter(
                logger,
                kinodynamics,
                RobotController::getBatteryVoltage);
        Command manual = new ManualMecanum(
                logger,
                driverControl::velocity,
                MAX_SPEED_X_M_S,
                MAX_SPEED_Y_M_S,
                MAX_OMEGA_RAD_S,
                limiter,
                m_drive);
        m_drive.setDefaultCommand(manual);

        m_autons = new Autons(logger, fieldLogger, m_drive);

        new Trigger(driverControl::back).onTrue(
                m_drive.resetPose());

        new Trigger(driverControl::x).whileTrue(
                m_drive.run(
                        () -> m_drive.setVelocity(new GlobalVelocityR3(1, 0, 0)))
                        .withTimeout(1.0));

        // an example of sequential commands.
        new Trigger(driverControl::y).whileTrue(
                sequence(
                        m_drive.driveWithGlobalVelocity(
                                new GlobalVelocityR3(1.5, 0, 0))
                                .withTimeout(1.0),
                        m_drive.driveWithGlobalVelocity(
                                new GlobalVelocityR3(0, 1.5, 0))
                                .withTimeout(1.0)));
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        Command auton = m_autons.get().command();
        if (auton == null)
            return;
        auton.schedule();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {
        checkStart();
        checkAlliance();
    }

    private void checkStart() {
        AnnotatedCommand cmd = m_autons.get();
        Pose2d start = cmd.start();
        if (start == null) {
            m_noStartingPosition.set(true);
        } else {
            m_noStartingPosition.set(false);
            m_drive.setPose(start);
        }
    }

    private void checkAlliance() {
        AnnotatedCommand cmd = m_autons.get();
        Alliance alliance = cmd.alliance();
        if (alliance == null) {
            // works for either
            return;
        }
        Alliance dsAlliance = DriverStation.getAlliance().orElse(null);
        if (dsAlliance == null) {
            // not set yet
            return;
        }
        if (alliance != dsAlliance) {
            m_mismatchedAlliance.set(true);
        } else {
            m_mismatchedAlliance.set(false);
        }
    }
}
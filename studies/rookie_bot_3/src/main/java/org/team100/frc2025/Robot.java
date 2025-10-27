package org.team100.frc2025;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.commands.tank.TankManual;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.motion.tank.TankDrive;
import org.team100.lib.motion.tank.TankDriveFactory;
import org.team100.lib.util.Banner;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Robot extends TimedRobot100 {
    private static final double MAX_SPEED_M_S = 3.0;
    private static final double MAX_OMEGA_RAD_S = 3.0;
    private final TankDrive m_drive;
    private final Command m_auton;

    public Robot() {
        Banner.printBanner();
        DriverStation.silenceJoystickConnectionWarning(true);
        Experiments.instance.show();
        SmartDashboard.putData(CommandScheduler.getInstance());
        Logging logging = Logging.instance();
        LoggerFactory fieldLogger = logging.fieldLogger;
        DriverXboxControl driverControl = new DriverXboxControl(0);
        LoggerFactory logger = logging.rootLogger;
        m_drive = TankDriveFactory.make(
                fieldLogger,
                logger,
                80, // stator current limit (a)
                new CanId(3), // left
                new CanId(27), // right
                0.4, // track width
                6.0, // gear ratio
                0.15); // wheel dia (m)
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.tank();
        SwerveLimiter limiter = new SwerveLimiter(
                logger,
                kinodynamics,
                RobotController::getBatteryVoltage);
        TankManual manual = new TankManual(
                logger,
                () -> -1.0 * driverControl.rightY(),
                () -> -1.0 * driverControl.rightX(),
                MAX_SPEED_M_S,
                MAX_OMEGA_RAD_S,
                limiter,
                m_drive);
        m_drive.setDefaultCommand(
                manual);

        new Trigger(driverControl::x).whileTrue(
                m_drive.run(() -> m_drive.setDutyCycle(1.0, 0.0))
                        .withTimeout(1.0));

        // make a sort of S-shape, as an example of sequential commands.
        new Trigger(driverControl::y).whileTrue(
                sequence(
                        m_drive.driveWithVelocity(1.5, 2.5).withTimeout(1.0),
                        m_drive.driveWithVelocity(1.5, -2.5).withTimeout(1.0)));

        m_auton = m_drive.run(() -> m_drive.setDutyCycle(1.0, 0.0))
                .withTimeout(1.0);
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        if (m_auton == null)
            return;
        m_auton.schedule();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}

package org.team100.frc2025;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.repeatingSequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.io.IOException;
import java.util.function.BooleanSupplier;

import org.team100.frc2025.shooter.DrumShooterFactory;
import org.team100.frc2025.shooter.IndexerServo;
import org.team100.frc2025.shooter.PivotDefault;
import org.team100.frc2025.shooter.PivotSubsystem;
import org.team100.frc2025.shooter.Shoot;
import org.team100.lib.examples.shooter.DualDrumShooter;
import org.team100.lib.examples.tank.DriveTank;
import org.team100.lib.examples.tank.TankDrive;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private final TankDrive m_drive;
    private final Command m_auton;
    private final DualDrumShooter m_shooter;
    private final IndexerServo m_indexer;
    private final PivotSubsystem m_pivot;

    public RobotContainer(TimedRobot100 robot) throws IOException {
        final Logging logging = Logging.instance();

        LoggerFactory fieldLogger = logging.fieldLogger;

        final LoggerFactory logger = logging.rootLogger;

        final DriverXboxControl driverControl = new DriverXboxControl(0);

        final LoggerFactory sysLog = logger.name("Subsystems");

        m_drive = TankFactory.make(fieldLogger, logger, 20);
        m_drive.setDefaultCommand(new DriveTank(driverControl::rightY, driverControl::rightX, m_drive));

        m_shooter = DrumShooterFactory.make(sysLog, 20);
        m_shooter.setDefaultCommand(m_shooter.run(m_shooter::stop));

        m_indexer = new IndexerServo(sysLog, 0);
        m_indexer.setDefaultCommand(m_indexer.run(m_indexer::stop));

        m_pivot = new PivotSubsystem(sysLog, 15);
        m_pivot.setDefaultCommand(new PivotDefault(driverControl::leftY, m_pivot));

        // this shows two ways to do the "shoot when spinning fast enough" thing.

        // a command class that contains the condition
        whileTrue(driverControl::a, new Shoot(m_shooter, m_indexer));

        // "fluent" command assembly.
        whileTrue(driverControl::y,
                parallel(
                        m_shooter.spin(),
                        repeatingSequence(
                                waitUntil(m_shooter::atGoal),
                                m_indexer.feed().withTimeout(0.5))));

        // whileTrue(driverControl::fullCycle, new ShootOne(m_shooter, m_indexer));
        whileTrue(driverControl::x, m_shooter.spin());

        m_auton = null;
    }

    public void onInit() {
    }

    public void onTeleopInit() {
        m_pivot.setEncoderPosition(Math.PI / 2);
        // new ZeroPivot(m_pivot).schedule();
    }

    public void periodic() {
    }

    public void onAuto() {
    }

    public void close() {
    }

    private void whileTrue(BooleanSupplier condition, Command command) {
        new Trigger(condition).whileTrue(command);
    }

    public void scheduleAuton() {
        if (m_auton == null)
            return;
        m_auton.schedule();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

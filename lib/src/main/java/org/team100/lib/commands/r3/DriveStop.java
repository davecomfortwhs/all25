package org.team100.lib.commands.r3;

import org.team100.lib.subsystems.SubsystemR3;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Stop the drivetrain.
 */
public class DriveStop extends Command {
    private final SubsystemR3 m_drive;

    public DriveStop(SubsystemR3 robotDrive) {
        m_drive = robotDrive;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.stop();
    }
}

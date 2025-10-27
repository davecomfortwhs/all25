package org.team100.lib.commands.swerve.manual;

import org.team100.lib.hid.Velocity;
import org.team100.lib.motion.swerve.SwerveDriveSubsystem;
import org.team100.lib.state.ModelR3;

public class ChassisSpeedAdapter implements DriverAdapter {
    private static final boolean DEBUG = false;

    private final SwerveDriveSubsystem m_drive;
    private final ChassisSpeedDriver m_driver;

    public ChassisSpeedAdapter(SwerveDriveSubsystem drive, ChassisSpeedDriver driver) {
        m_drive = drive;
        m_driver = driver;
    }

    @Override
    public void apply(ModelR3 s, Velocity t) {
        if (DEBUG) {
            System.out.printf("ChassisSpeedDriver %s\n", t);
        }
        m_drive.setChassisSpeeds(m_driver.apply(s, t));
    }

    @Override
    public void reset(ModelR3 p) {
        m_driver.reset(p);
    }
}

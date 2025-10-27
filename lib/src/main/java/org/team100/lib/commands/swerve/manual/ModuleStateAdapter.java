package org.team100.lib.commands.swerve.manual;

import org.team100.lib.hid.Velocity;
import org.team100.lib.motion.swerve.SwerveDriveSubsystem;
import org.team100.lib.state.ModelR3;

public class ModuleStateAdapter implements DriverAdapter {
    private static final boolean DEBUG = false;

    private final SwerveDriveSubsystem m_drive;
    private final ModuleStateDriver m_driver;

    public ModuleStateAdapter(SwerveDriveSubsystem drive, ModuleStateDriver driver) {
        m_drive = drive;
        m_driver = driver;
    }

    @Override
    public void apply(ModelR3 s, Velocity t) {
        if (DEBUG) {
            System.out.printf("ModuleStateDriver %s\n",  t );
        }
        m_drive.setRawModuleStates(m_driver.apply(t));
    }

    @Override
    public void reset(ModelR3 p) {
        //
    }
}

package org.team100.lib.config;

import org.team100.lib.util.NamedChooser;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonChooser {

    private final NamedChooser<AnnotatedCommand> m_chooser = new NamedChooser<>("Auton Command");

    public AutonChooser() {
        m_chooser.setDefaultOption("NONE", new AnnotatedCommand(
                null, null, null));
        SmartDashboard.putData(m_chooser);
    }

    public void addAsDefault(String name, AnnotatedCommand cmd) {
        m_chooser.setDefaultOption(name, cmd);
    }

    public void add(String name, AnnotatedCommand command) {
        m_chooser.addOption(name, command);
    }

    public AnnotatedCommand get() {
        return m_chooser.getSelected();
    }

    public void close() {
        m_chooser.close();
    }
}

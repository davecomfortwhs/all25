package org.team100.lib.subsystems;

import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.state.ModelR3;

public class MockSubsystemR3 implements SubsystemR3 {
    public GlobalVelocityR3 m_setpoint;
    public GlobalVelocityR3 m_recentSetpoint;
    public ModelR3 m_state;

    public MockSubsystemR3(ModelR3 initial) {
        m_state = initial;
    }

    @Override
    public ModelR3 getState() {
        return m_state;
    }

    @Override
    public void stop() {
        // do nothing
    }

    @Override
    public void setVelocity(GlobalVelocityR3 setpoint) {
        m_setpoint = setpoint;
        m_recentSetpoint = setpoint;
    }

}

package org.team100.lib.reference.r1;

import org.team100.lib.coherence.Takt;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.profile.timed.TimedProfile;
import org.team100.lib.state.Model100;

/**
 * Extracts current and next references from a timed profile.
 */
public class TimedProfileReferenceR1 implements ProfileReferenceR1 {

    private final TimedProfile m_profile;
    private Model100 m_goal;
    private double m_startTimeS;

    public TimedProfileReferenceR1(TimedProfile profile) {
        m_profile = profile;
    }

    @Override
    public void setGoal(Model100 goal) {
        m_goal = goal;
    }

    @Override
    public void init(Model100 measurement) {
        if (m_goal == null)
            throw new IllegalStateException("goal must be set");
        m_startTimeS = Takt.get();
        m_profile.init(measurement.control(), m_goal);
    }

    @Override
    public SetpointsR1 get() {
        double progress = progress();
        return new SetpointsR1(
                m_profile.sample(progress),
                m_profile.sample(progress + TimedRobot100.LOOP_PERIOD_S));
    }

    @Override
    public boolean profileDone() {
        return progress() >= duration();
    }

    //////////////////////////////////////////////////////////////////////////////////////

    private double duration() {
        return m_profile.duration();
    }

    private double progress() {
        return Takt.get() - m_startTimeS;
    }

}

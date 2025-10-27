package org.team100.lib.reference.r3;

import org.team100.lib.coherence.Takt;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.trajectory.Trajectory100;

/** Produces references based on a trajectory. */
public class TrajectoryReferenceR3 implements ReferenceR3 {
    private final Trajectory100 m_trajectory;
    private double m_startTimeS;

    public TrajectoryReferenceR3(Trajectory100 trajectory) {
        m_trajectory = trajectory;
    }

    /** Ignores the measurement, resets the trajectory timer. */
    @Override
    public void initialize(ModelR3 measurement) {
        m_startTimeS = Takt.get();
    }

    @Override
    public ModelR3 current() {
        return sample(progress()).model();
    }

    @Override
    public ControlR3 next() {
        return sample(progress() + TimedRobot100.LOOP_PERIOD_S);
    }

    @Override
    public boolean done() {
        return m_trajectory.isDone(progress());
    }

    @Override
    public ModelR3 goal() {
        return ControlR3.fromTimedPose(m_trajectory.getLastPoint()).model();
    }

    ////////////////////////////////////////////////////

    private double progress() {
        return Takt.get() - m_startTimeS;
    }

    private ControlR3 sample(double t) {
        return ControlR3.fromTimedPose(m_trajectory.sample(t));
    }
}

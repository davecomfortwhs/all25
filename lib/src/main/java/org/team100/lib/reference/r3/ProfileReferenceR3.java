package org.team100.lib.reference.r3;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.CotemporalCache;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;

/**
 * Produces references based on profiles.
 * 
 * This uses the central Cache to manage reference updates, because it's
 * important that the reference updates be aligned with the clock, i.e. one
 * profile update step per Takt step, but the SwerveReference interface doesn't
 * know anything about time. Still, it's a sort of strange thing to do,
 * since it's not a "Cache" per se.
 * 
 * One of the implications of this choice is that the cache updates will
 * continue forever unless we explicitly stop them, so if you use this class,
 * you need to remember to do that.
 */
public class ProfileReferenceR3 implements ReferenceR3 {
    private static final boolean DEBUG = false;
    private static final double TOLERANCE = 0.01;

    /**
     * Putting these in the same class allows us to refresh them both atomically.
     */
    private record References(ModelR3 m_current, ControlR3 m_next) {
    }

    private final HolonomicProfile m_profile;
    /** The name is for debugging. */
    private final String m_name;
    private final CotemporalCache<References> m_references;

    private ModelR3 m_goal;
    private boolean m_done;

    /**
     * The most-recently calculated "next" reference, which will be a future
     * "current" reference.
     */
    private ControlR3 m_next;

    public ProfileReferenceR3(HolonomicProfile profile, String name) {
        m_profile = profile;
        m_name = name;
        // this will keep polling until we stop it.
        m_references = Cache.of(() -> refresh(m_next == null ? null : m_next.model()));
    }

    /**
     * This does not solve for profile coordination, so if you update the goal after
     * initialize(), you'll use the old scales. That's probably fine, if the goal
     * hasn't moved much, but it's not appropriate to move the goal a lot.
     */
    public void setGoal(ModelR3 goal) {
        m_goal = goal;
    }

    /** Immediately overwrite the references. */
    @Override
    public void initialize(ModelR3 measurement) {
        m_profile.solve(measurement, m_goal);
        m_references.set(refresh(measurement));
        m_done = false;
    }

    @Override
    public ModelR3 current() {
        return m_references.get().m_current;
    }

    @Override
    public ControlR3 next() {
        return m_references.get().m_next;
    }

    @Override
    public boolean done() {
        return m_done;
    }

    @Override
    public ModelR3 goal() {
        return m_goal;
    }

    /**
     * Stop updating the references. There's no way to restart the updates, so you
     * should discard this object once you call end().
     */
    public void end() {
        m_references.end();
    }

    ////////////////////////////////////

    private References refresh(ModelR3 newCurrent) {
        m_next = makeNext(newCurrent);
        return new References(newCurrent, m_next);
    }

    private ControlR3 makeNext(ModelR3 current) {
        if (DEBUG) {
            System.out.printf("ProfileReference refreshing %s\n", m_name);
        }
        if (current == null) {
            // happens at startup
            return null;
        }
        if (m_goal == null) {
            return current.control();
        }
        ControlR3 next = m_profile.calculate(current, m_goal);
        if (current.near(m_goal, TOLERANCE))
            m_done = true;
        return next;
    }

}

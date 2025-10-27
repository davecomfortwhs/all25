package org.team100.lib.reference.r3;

import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;

/** Always returns the same reference. */
public class ConstantReferenceR3 implements ReferenceR3 {
    private final ModelR3 m_goal;

    public ConstantReferenceR3(ModelR3 goal) {
        m_goal = goal;
    }

    @Override
    public void initialize(ModelR3 measurement) {
        //
    }

    @Override
    public ModelR3 current() {
        return m_goal;
    }

    @Override
    public ControlR3 next() {
        return m_goal.control();
    }

    @Override
    public boolean done() {
        return false;
    }

    @Override
    public ModelR3 goal() {
        return m_goal;
    }

}

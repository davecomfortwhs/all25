package org.team100.lib.reference;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.profile.timed.JerkLimitedTimedProfile;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.SetpointsR1;
import org.team100.lib.reference.r1.TimedProfileReferenceR1;
import org.team100.lib.state.Model100;
import org.team100.lib.testing.Timeless;

public class TimedProfileReference1dTest implements Timeless {
    private static final double DELTA = 0.001;

    @Test
    void testSimple() {
        JerkLimitedTimedProfile p = new JerkLimitedTimedProfile(2, 6, 25, false);
        Model100 goal = new Model100(1, 0);
        ProfileReferenceR1 ref = new TimedProfileReferenceR1(p);
        ref.setGoal(goal);
        Model100 measurement = new Model100();
        ref.init(measurement);

        // initial current setpoint is the measurement.
        SetpointsR1 s = ref.get();
        assertEquals(0, s.current().x(), DELTA);
        assertEquals(0, s.current().v(), DELTA);
        assertEquals(0, s.current().a(), DELTA);
        assertEquals(0.00003, s.next().x(), DELTA);
        assertEquals(0.005, s.next().v(), DELTA);
        assertEquals(0.500, s.next().a(), DELTA);

        // if time does not pass, nothing changes.
        s = ref.get();
        assertEquals(0, s.current().x(), DELTA);
        assertEquals(0, s.current().v(), DELTA);
        assertEquals(0, s.current().a(), DELTA);
        assertEquals(0.00003, s.next().x(), DELTA);
        assertEquals(0.005, s.next().v(), DELTA);
        assertEquals(0.500, s.next().a(), DELTA);

        stepTime();

        // now the setpoint has advanced
        s = ref.get();
        assertEquals(0, s.current().x(), DELTA);
        assertEquals(0.005, s.current().v(), DELTA);
        assertEquals(0.500, s.current().a(), DELTA);
        assertEquals(0.00026, s.next().x(), DELTA);
        assertEquals(0.020, s.next().v(), DELTA);
        assertEquals(1.000, s.next().a(), DELTA);

    }

}

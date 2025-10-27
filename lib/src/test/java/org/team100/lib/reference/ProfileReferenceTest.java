package org.team100.lib.reference;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.reference.r3.ProfileReferenceR3;
import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ProfileReferenceTest implements Timeless {
    private static final double DELTA = 0.001;

    @Test
    void testSimple() {
        ModelR3 measurement = new ModelR3(new Pose2d(0, 0, Rotation2d.kZero));
        ModelR3 goal = new ModelR3(new Pose2d(1, 0, Rotation2d.kZero));
        HolonomicProfile hp = HolonomicProfile.trapezoidal(1, 1, 0.01, 1, 1, 0.01);
        ProfileReferenceR3 r = new ProfileReferenceR3(hp, "test");
        r.setGoal(goal);
        r.initialize(measurement);
        {
            ModelR3 c = r.current();
            assertEquals(0, c.velocity().x(), DELTA);
            assertEquals(0, c.pose().getX(), DELTA);
            ControlR3 n = r.next();
            assertEquals(0.02, n.velocity().x(), DELTA);
            assertEquals(0, n.pose().getX(), DELTA);
        }
        // no time step, nothing changes
        {
            ModelR3 c = r.current();
            assertEquals(0, c.velocity().x(), DELTA);
            assertEquals(0, c.pose().getX(), DELTA);
            ControlR3 n = r.next();
            assertEquals(0.02, n.velocity().x(), DELTA);
            // x is very small but not zero
            assertEquals(0.0002, n.pose().getX(), 0.00001);
        }
        // stepping time gets the next references
        stepTime();
        {
            ModelR3 c = r.current();
            assertEquals(0.02, c.velocity().x(), DELTA);
            assertEquals(0, c.pose().getX(), DELTA);
            ControlR3 n = r.next();
            assertEquals(0.04, n.velocity().x(), DELTA);
            assertEquals(0.00078, n.pose().getX(), DELTA);
        }
        // way in the future, we're at the end.
        for (int i = 0; i < 500; ++i) {
            stepTime();
        }
        {
            ModelR3 c = r.current();
            assertEquals(0, c.velocity().x(), DELTA);
            assertEquals(1, c.pose().getX(), DELTA);
            ControlR3 n = r.next();
            assertEquals(0, n.velocity().x(), DELTA);
            assertEquals(1, n.pose().getX(), DELTA);
        }
    }

}

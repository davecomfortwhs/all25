package org.team100.lib.reference;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.coherence.Cache;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.reference.r3.TrajectoryReferenceR3;
import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.TimingConstraintFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TrajectoryReferenceTest implements Timeless {
    private static final double DELTA = 0.001;
    SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood(logger);
    TrajectoryPlanner planner = new TrajectoryPlanner(constraints);

    @Test
    void testSimple() {
        Cache.clear();
        Trajectory100 t = planner.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
        TrajectoryReferenceR3 r = new TrajectoryReferenceR3(t);
        // measurement is irrelevant
        r.initialize(new ModelR3());
        {
            ModelR3 c = r.current();
            assertEquals(0, c.velocity().x(), DELTA);
            assertEquals(0, c.pose().getX(), DELTA);
            ControlR3 n = r.next();
            assertEquals(0.033, n.velocity().x(), DELTA);
            assertEquals(0, n.pose().getX(), DELTA);
        }
        // no time step, nothing changes
        {
            ModelR3 c = r.current();
            assertEquals(0, c.velocity().x(), DELTA);
            assertEquals(0, c.pose().getX(), DELTA);
            ControlR3 n = r.next();
            assertEquals(0.033, n.velocity().x(), DELTA);
            // x is very small but not zero
            assertEquals(0.0003266, n.pose().getX(), 0.0000001);
        }
        // stepping time gets the next references
        stepTime();
        {
            ModelR3 c = r.current();
            assertEquals(0.033, c.velocity().x(), DELTA);
            assertEquals(0, c.pose().getX(), DELTA);
            ControlR3 n = r.next();
            assertEquals(0.065, n.velocity().x(), DELTA);
            assertEquals(0.001, n.pose().getX(), DELTA);
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

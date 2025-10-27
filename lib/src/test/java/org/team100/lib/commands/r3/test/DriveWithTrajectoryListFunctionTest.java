package org.team100.lib.commands.r3.test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.List;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.team100.lib.controller.r3.ControllerFactoryR3;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.swerve.Fixtured;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.TimingConstraintFactory;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj.DataLogManager;

class DriveWithTrajectoryListFunctionTest extends Fixtured implements Timeless {
    public DriveWithTrajectoryListFunctionTest() throws IOException {
    }

    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final TrajectoryVisualization viz = new TrajectoryVisualization(logger);

    List<TimingConstraint> constraints = new TimingConstraintFactory(fixture.swerveKinodynamics).allGood(logger);
    TrajectoryPlanner planner = new TrajectoryPlanner(constraints);

    @BeforeEach
    void nolog() {
        DataLogManager.stop();
    }

    @Test
    void testSimple() {
        // this initial step is required since the timebase is different?
        stepTime();
        Experiments.instance.testOverride(Experiment.UseSetpointGenerator, true);
        ControllerR3 control = ControllerFactoryR3.test(logger);
        DriveWithTrajectoryListFunction c = new DriveWithTrajectoryListFunction(
                fixture.drive,
                control,
                x -> List.of(planner.line(x)),
                viz);
        c.initialize();
        assertEquals(0, fixture.drive.getPose().getX(), DELTA);
        c.execute();
        assertFalse(c.isDone());
        // the trajectory takes a little over 3s
        for (double t = 0; t < 3.2; t += TimedRobot100.LOOP_PERIOD_S) {
            stepTime();
            c.execute();
            fixture.drive.periodic(); // for updateOdometry
        }
        assertTrue(c.isDone());
        assertEquals(1.0, fixture.drive.getPose().getX(), 0.001);
    }
}

package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Camera;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ModelR3;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Translation2d;

/** Timeless because the clock is used to decide to ignore (stale) input. */
public class SimulatedTargetWriterTest implements Timeless {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(
            new TestPrimitiveLogger());
    private static final FieldLogger.Log fieldLog = new FieldLogger.Log(logger);

    @Test
    void testOne() {
        stepTime();
        ModelR3 p = new ModelR3();
        SimulatedTargetWriter writer = new SimulatedTargetWriter(
                List.of(Camera.TEST4),
                x -> p,
                new Translation2d[] {
                        new Translation2d(1, 0) });

        // need to instantiate the reader prior to the writer update because the poller
        // ignores things that came before.
        Targets reader = new Targets(logger, fieldLog, x -> p);

        stepTime();
        writer.update();
        stepTime();
        reader.update();

        assertEquals(1, reader.getTargets().size());
        Translation2d target = reader.getTargets().get(0);
        // camera is 1m up, tilted 45 down, so target is 1m away
        assertEquals(1.0, target.getX(), DELTA);
        // target is on bore
        assertEquals(0, target.getY(), DELTA);

        writer.close();

    }

}

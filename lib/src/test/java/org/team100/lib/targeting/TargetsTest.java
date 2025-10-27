package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Camera;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ModelR3;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;

/**
 * Timeless because the clock is used to decide to ignore (stale) input.
 */
public class TargetsTest implements Timeless {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(
            new TestPrimitiveLogger());
    private static final FieldLogger.Log fieldLog = new FieldLogger.Log(logger);

    @Test
    void testTargets() {
        stepTime();
        ModelR3 p = new ModelR3();
        Targets t = new Targets(logger, fieldLog, (x) -> p);
        t.update();
        assertTrue(t.getTargets().isEmpty());
        // send some blips
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("tag_finder24");

        // test4 camera offset is 0,0,1, without rotation
        StructArrayTopic<Rotation3d> topic = inst.getStructArrayTopic(
                "objectVision/test4/5678/Rotation3d", Rotation3d.struct);
        StructArrayPublisher<Rotation3d> pub = topic.publish();
        stepTime();
        // tilt down 45
        pub.set(new Rotation3d[] { new Rotation3d(0, Math.PI / 4, 0) },
                (long) (Takt.get() * 1000000.0));
        stepTime();
        t.update();
        assertEquals(1, t.getTargets().size());
        Translation2d target = t.getTargets().get(0);
        // camera is 1m up, tilted 45 down, so target is 1m away
        assertEquals(1.0, target.getX(), DELTA);
        // target is on bore
        assertEquals(0, target.getY(), DELTA);

    }

    @Test
    void testTranslations() {
        stepTime();

        ModelR3 p = new ModelR3();
        SimulatedTargetWriter writer = new SimulatedTargetWriter(
                List.of(Camera.TEST4),
                x -> p,
                new Translation2d[] { new Translation2d(1, 0) });

        // need to instantiate the reader prior to the writer update because the poller
        // ignores things that came before.
        Targets reader = new Targets(logger, fieldLog, (x) -> p);

        stepTime();
        writer.update();

        stepTime();
        reader.update();

        List<Translation2d> allTargets = reader.getTargets();
        assertEquals(1, allTargets.size());

        Optional<Translation2d> tt = reader.getClosestTarget();
        assertTrue(tt.isPresent());
        Translation2d ttt = tt.get();
        assertEquals(1.0, ttt.getX(), DELTA);
        assertEquals(0, ttt.getY(), DELTA);

        writer.close();
    }

    @Test
    void testMultipleCameras() {
        stepTime();
        ModelR3 p = new ModelR3();

        SimulatedTargetWriter writer = new SimulatedTargetWriter(
                List.of(Camera.TEST4, Camera.TEST5),
                x -> p,
                new Translation2d[] { new Translation2d(1, 0) });

        // need to instantiate the reader prior to the writer update because the poller
        // ignores things that came before.
        Targets reader = new Targets(logger, fieldLog, (x) -> p);

        stepTime();
        writer.update();

        stepTime();
        reader.update();

        List<Translation2d> allTargets = reader.getTargets();
        // both cameras see the sme target
        assertEquals(1, allTargets.size());

        Optional<Translation2d> tt = reader.getClosestTarget();
        assertTrue(tt.isPresent());
        Translation2d ttt = tt.get();
        assertEquals(1.0, ttt.getX(), DELTA);
        assertEquals(0, ttt.getY(), DELTA);

        writer.close();
    }

    @Test
    void testMultipleTargets() {
        stepTime();

        ModelR3 p = new ModelR3();
        SimulatedTargetWriter writer = new SimulatedTargetWriter(
                List.of(Camera.TEST4),
                x -> p,
                new Translation2d[] {
                        new Translation2d(1, 0),
                        new Translation2d(2, 0) });

        // need to instantiate the reader prior to the writer update because the poller
        // ignores things that came before.
        Targets reader = new Targets(logger, fieldLog, x -> p);

        stepTime();
        writer.update();

        stepTime();
        reader.update();

        List<Translation2d> allTargets = reader.getTargets();
        assertEquals(2, allTargets.size());

        Optional<Translation2d> tt = reader.getClosestTarget();
        assertTrue(tt.isPresent());
        Translation2d ttt = tt.get();
        assertEquals(1.0, ttt.getX(), DELTA);
        assertEquals(0, ttt.getY(), DELTA);

        writer.close();
    }

    @Test
    void testMultipleTargetsAndCameras() {
        stepTime();
        ModelR3 p = new ModelR3();

        SimulatedTargetWriter writer = new SimulatedTargetWriter(
                List.of(Camera.TEST4, Camera.TEST5),
                x -> p,
                new Translation2d[] {
                        new Translation2d(1, 0),
                        new Translation2d(2, 0) });

        // need to instantiate the reader prior to the writer update because the poller
        // ignores things that came before.
        Targets reader = new Targets(logger, fieldLog, (x) -> p);

        stepTime();
        writer.update();

        stepTime();
        reader.update();

        List<Translation2d> allTargets = reader.getTargets();
        // multi-camera views of the same target are coalesced
        assertEquals(2, allTargets.size());

        Optional<Translation2d> tt = reader.getClosestTarget();
        assertTrue(tt.isPresent());
        Translation2d ttt = tt.get();
        assertEquals(1.0, ttt.getX(), DELTA);
        assertEquals(0, ttt.getY(), DELTA);

        writer.close();
    }
}

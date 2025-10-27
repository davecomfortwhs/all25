package org.team100.lib.targeting;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleFunction;

import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Camera;
import org.team100.lib.field.FieldConstants;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.state.ModelR3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Write simulated targets to Network Tables, so the Targets receiver can pick
 * them up.
 */
public class SimulatedTargetWriter {
    private static final boolean DEBUG = false;

    // camera frame is from 85 ms ago
    private static final double DELAY = 0.085;

    private final Map<Camera, StructArrayPublisher<Rotation3d>> m_publishers;
    private final List<Camera> m_cameras;
    private final DoubleFunction<ModelR3> m_history;

    /** For now, a fixed list of targets */
    private final Translation2d[] m_targets;
    /** client instance, not the default */
    private final NetworkTableInstance m_inst;

    public SimulatedTargetWriter(
            List<Camera> cameras,
            DoubleFunction<ModelR3> history,
            Translation2d[] targets) {
        m_cameras = cameras;
        m_history = history;
        m_targets = targets;
        m_publishers = new HashMap<>();
        m_inst = NetworkTableInstance.getDefault();
        m_inst.startClient4("tag_finder24");
        m_inst.setServer("localhost");
        for (Camera camera : m_cameras) {
            String name = "objectVision/"
                    + camera.getSerial() + "/0/Rotation3d";
            m_publishers.put(
                    camera,
                    m_inst.getStructArrayTopic(
                            name, Rotation3d.struct).publish());
        }
    }

    public static Runnable get(SwerveHistory history) {
        if (RobotBase.isReal()) {
            // Real robots get an empty simulated target detector.
            return () -> {
            };
        }
        // In simulation, we want the real simulated target detector.
        SimulatedTargetWriter tsim = new SimulatedTargetWriter(
                List.of(Camera.SWERVE_LEFT,
                        Camera.SWERVE_RIGHT,
                        Camera.FUNNEL,
                        Camera.CORAL_LEFT,
                        Camera.CORAL_RIGHT),
                history,
                new Translation2d[] {
                        FieldConstants.CoralMark.LEFT.value,
                        FieldConstants.CoralMark.CENTER.value,
                        FieldConstants.CoralMark.RIGHT.value });
        return tsim::update;
    }

    public void update() {
        if (DEBUG)
            System.out.println("simulated target write update");
        // select pose from a little while ago
        double timestampS = Takt.get() - DELAY;
        Pose2d pose = m_history.apply(timestampS).pose();

        for (Map.Entry<Camera, StructArrayPublisher<Rotation3d>> entry : m_publishers.entrySet()) {
            Camera camera = entry.getKey();
            StructArrayPublisher<Rotation3d> publisher = entry.getValue();
            List<Rotation3d> rot = SimulatedObjectDetector.getRotations(
                    pose, camera.getOffset(), m_targets);
            if (DEBUG) {
                System.out.printf("rot size %d\n", rot.size());
            }
            // tilt down 45
            // Rotation3d[] rots = new Rotation3d[] { new Rotation3d(0, Math.PI / 4, 0) };
            Rotation3d[] rots = rot.toArray(new Rotation3d[0]);

            // write timestamp corresponding to pose
            long delayUs = (long) (DELAY * 1000000.0);
            long timestampUs = NetworkTablesJNI.now();
            // long timestampUs = (long)(Takt.get() * 1000000.0);

            long time = timestampUs - delayUs;
            if (DEBUG) {
                System.out.printf("writer timestamp %d\n", time);
            }
            publisher.set(rots, time);
        }
    }

    public void close() {
        m_publishers.values().forEach(p -> p.close());
    }
}

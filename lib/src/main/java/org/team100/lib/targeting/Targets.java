package org.team100.lib.targeting;

import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;
import java.util.function.Function;
import java.util.stream.DoubleStream;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.SideEffect;
import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.IntLogger;
import org.team100.lib.network.CameraReader;
import org.team100.lib.state.ModelR3;
import org.team100.lib.util.CoalescingCollection;
import org.team100.lib.util.TrailingHistory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.StructBuffer;

/**
 * Listen for updates from the object-detector camera and remember them for
 * awhile.
 */
public class Targets extends CameraReader<Rotation3d> {
    public static class Mean implements Function<Collection<Translation2d>, Translation2d> {
        @Override
        public Translation2d apply(Collection<Translation2d> c) {
            Translation2d sum = new Translation2d();
            int count = 0;
            for (Translation2d t : c) {
                sum = sum.plus(t);
                count++;
            }
            return sum.div(count);
        }
    }

    private static final boolean DEBUG = false;

    /**
     * Ignore incoming sights older than this, because they're stale.
     * This shouldn't happen if the camera is working well.
     */
    private static final double MAX_SIGHT_AGE = 0.2;
    /** Forget sights older than this. */
    private static final double HISTORY_DURATION = 1.0;
    /** Targets closer than this to each other are combined */
    private static final double RESOLUTION = 0.15;

    private final FieldLogger.Log m_field_log;

    /** state = f(takt seconds) from history. */
    private final DoubleFunction<ModelR3> m_history;
    /** Accumulation of targets we see. */
    // private final TrailingHistory<Translation2d> m_targets;
    private final CoalescingCollection<Translation2d> m_targets;
    /** Side effect mutates targets. */
    private final SideEffect m_vision;
    private final IntLogger m_log_historySize;

    public Targets(
            LoggerFactory log,
            FieldLogger.Log fieldLogger,
            DoubleFunction<ModelR3> history) {
        super(
                "objectVision",
                "Rotation3d",
                StructBuffer.create(Rotation3d.struct));
        m_log_historySize = log.type(this).intLogger(Level.TRACE, "history size");
        m_field_log = fieldLogger;
        m_history = history;
        // m_targets = new TrailingHistory<>(HISTORY_DURATION);
        m_targets = new CoalescingCollection<>(
                new TrailingHistory<>(HISTORY_DURATION),
                (a, b) -> a.getDistance(b) < RESOLUTION,
                new Mean());
        m_vision = Cache.ofSideEffect(this::update);
    }

    @Override
    protected void perValue(
            Transform3d cameraOffset,
            double valueTimestamp,
            Rotation3d[] sights) {
        double age = Takt.get() - valueTimestamp;
        if (age > MAX_SIGHT_AGE) {
            if (DEBUG) {
                System.out.printf("WARNING: ignoring stale sight %f %f\n", Takt.get(), valueTimestamp);
            }
            return;
        }
        Pose2d robotPose = m_history.apply(valueTimestamp).pose();
        m_targets.addAll(
                valueTimestamp,
                TargetLocalizer.cameraRotsToFieldRelativeArray(
                        robotPose,
                        cameraOffset,
                        sights));
    }

    /**
     * Field-relative translations of recent sights.
     */
    public List<Translation2d> getTargets() {
        m_vision.run();
        return m_targets.getAll();
    }

    /**
     * The field-relative translation of the closest object, if any.
     */
    public Optional<Translation2d> getClosestTarget() {
        Pose2d robotPose = m_history.apply(Takt.get()).pose();
        List<Translation2d> targets = getTargets();
        if (DEBUG) {
            System.out.printf("translations %d\n", targets.size());
        }
        return ObjectPicker.closestObject(targets, robotPose);
    }

    public void periodic() {
        // show the closest target we can see on the field2d widget.
        // getClosestTarget().ifPresent(
        // x -> m_field_log.m_log_target.log(
        // () -> new double[] { x.getX(), x.getY(), 0 }));

        // show *all* targets on the field2d widget.
        m_field_log.m_log_target.log(
                () -> getTargets().stream().flatMapToDouble(
                        x -> DoubleStream.of(x.getX(), x.getY(), 0.0)).toArray());

        m_log_historySize.log(() -> m_targets.size());
    }
}
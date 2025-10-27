package org.team100.lib.tuning;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.DoubleCache;
import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * A mutable double linked to Network Tables, for tuning.
 * 
 * Use the DoubleSupplier API for polling.
 * Provide a DoubleConsumer to get called on changes.
 * Values do not survive restarts.
 */
public class Mutable implements DoubleSupplier {
    private final DoubleEntry m_entry;
    private final DoubleConsumer m_onChange;
    private final DoubleCache m_cache;

    public Mutable(LoggerFactory log, String leaf, double defaultValue, DoubleConsumer onChange) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startServer();
        DoubleTopic topic = inst.getDoubleTopic(log.root(leaf));
        m_entry = topic.getEntry(defaultValue);
        // Makes sure we don't get a stale value.
        m_entry.set(defaultValue);
        // You can't use "persistent" here, because then the key goes in the RoboRIO
        // networktables.json file and can never be deleted (except manually).
        topic.setRetained(true);
        m_entry.readQueue();
        m_onChange = onChange;
        m_cache = Cache.ofDouble(this::update);
    }

    /** if you don't care to subscribe to changes */
    public Mutable(LoggerFactory log, String leaf, double defaultValue) {
        this(log, leaf, defaultValue, (x) -> {
        });
    }

    /** Supply the current value. */
    @Override
    public double getAsDouble() {
        return m_cache.getAsDouble();
    }

    /** Updates the cache and also notifies the consumer. */
    private double update() {
        double[] queue = m_entry.readQueueValues();
        double val = m_entry.get();
        if (queue.length > 0)
            m_onChange.accept(val);
        return val;
    }

}

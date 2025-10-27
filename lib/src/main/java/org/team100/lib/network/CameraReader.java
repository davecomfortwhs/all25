package org.team100.lib.network;

import java.util.EnumSet;

import org.team100.lib.config.Camera;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.MultiSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListenerPoller;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.ValueEventData;
import edu.wpi.first.util.struct.StructBuffer;

/**
 * Reads camera input from network tables, which is always a StructArray.
 * 
 * @param T payload type
 */
public abstract class CameraReader<T> {
    private static final boolean DEBUG = false;
    /**
     * Five cameras, 50hz each => 250 hz of updates. Rio runs at 50 hz, so there
     * should be five messages waiting for us each cycle.
     */
    private static final int QUEUE_DEPTH = 10;

    /** e.g. "blips" or "Rotation3d" */
    private final String m_ntValueName;
    /** Manages the queue of incoming messages. */
    private final NetworkTableListenerPoller m_poller;
    /** Deserializer used in update(). */
    private final StructBuffer<T> m_buf;

    public CameraReader(
            String ntRootName,
            String ntValueName,
            StructBuffer<T> buf) {
        m_ntValueName = ntValueName;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_poller = new NetworkTableListenerPoller(inst);
        m_poller.addListener(
                new MultiSubscriber(
                        inst,
                        new String[] { ntRootName },
                        PubSubOption.keepDuplicates(true),
                        PubSubOption.pollStorage(QUEUE_DEPTH)),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll));
        m_buf = buf;
    }

    /**
     * Read queued network input, and give it to the consumers.
     * 
     * This runs once per cycle, in SwerveDriveSubsystem.update() which is called by
     * Memo.updateAll(), which runs in Robot.robotPeriodic().
     */
    public void update() {
        beginUpdate();
        for (NetworkTableEvent e : m_poller.readQueue()) {
            ValueEventData valueEventData = e.valueData;
            NetworkTableValue ntValue = valueEventData.value;
            String name = valueEventData.getTopic().getName();
            if (DEBUG) {
                System.out.printf("poll %s\n", name);
            }
            String[] fields = name.split("/");
            if (fields.length != 4) {
                System.out.printf("WARNING: weird event name: %s\n", name);
                continue;
            }
            // key is "rootName/cameraId/cameraNumber/valueName"
            String cameraId = fields[1];
            if (!fields[3].equals(m_ntValueName)) {
                System.out.println("WARNING: weird key: " + name);
                continue;
            }
            if (DEBUG) {
                System.out.print("found value\n");
            }
            // decode the way StructArrayEntryImpl does
            byte[] valueBytes = ntValue.getRaw();
            if (valueBytes.length == 0) {
                // this should never happen, but it does, very occasionally.
                continue;
            }
            T[] valueArray;
            try {
                valueArray = m_buf.readArray(valueBytes);
            } catch (RuntimeException ex) {
                System.out.printf("WARNING: decoding failed for name: %s\n", name);
                continue;
            }

            // Robot-to-camera, offset from Camera.java
            // in tests this offset is identity.
            Transform3d cameraOffset = Camera.get(cameraId).getOffset();
            if (DEBUG) {
                System.out.printf("camera %s offset %s\n", cameraId, cameraOffset);
            }

            // server time is in microseconds
            // https://docs.wpilib.org/en/stable/docs/software/networktables/networktables-intro.html#timestamps
            //
            // ATTENTION! (sep 15 2025)
            //
            // using server time seems to break the tests, like server time ignores the test
            // clock, which makes me wonder if it's just the wrong thing to use all the
            // time, so this uses "local" time now.
            // double valueTimestamp = ((double)ntValue.getServerTime()) / 1000000.0;
            double valueTimestamp = ((double) ntValue.getTime()) / 1000000.0;
            if (DEBUG) {
                System.out.printf("reader timestamp %f\n", valueTimestamp);
            }

            perValue(cameraOffset, valueTimestamp, valueArray);
        }
        finishUpdate();
    }

    /** Called when update() starts. */
    protected void beginUpdate() {
    };

    /**
     * Called for each StructArray received.
     * 
     * @param cameraOffset   camera pose in robot coordinates
     * @param valueTimestamp network tables local time in seconds
     * @param valueArray     payload array
     */
    protected abstract void perValue(
            Transform3d cameraOffset,
            double valueTimestamp,
            T[] value);

    /** Called when update() ends. */
    protected void finishUpdate() {
    }

}

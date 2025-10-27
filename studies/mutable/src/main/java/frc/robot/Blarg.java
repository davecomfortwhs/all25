package frc.robot;

import java.util.function.DoubleSupplier;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.tuning.Mutable;

/**
 * Demo of Mutable.
 */
public class Blarg {
    private final DoubleSupplier m_mutableThing;

    public Blarg(LoggerFactory root) {
        m_mutableThing = new Mutable(root, "bar", 0.0, this::onChange);
    }

    /**
     * This is OK for things that don't take time, like just using the value
     * locally.
     */
    public double get() {
        return m_mutableThing.getAsDouble();
    }

    /**
     * This is good for things that take time, like updating outboard parameters.
     */
    public void onChange(double newValue) {
        System.out.printf("new value %6.3f\n", newValue);
    }

}

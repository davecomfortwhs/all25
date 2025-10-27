package org.team100.lib.motion.swerve;

import java.io.IOException;

import org.junit.jupiter.api.AfterEach;
import org.team100.lib.testing.Timeless;

/**
 * Test superclass with fixture.
 * Uses simulated position sensors, must be used with clock control (e.g.
 * {@link Timeless}).
 */
public class Fixtured {
    protected final Fixture fixture;

    public Fixtured() throws IOException {
        fixture = new Fixture();
    }

    @AfterEach
    void close() {
        fixture.close();
    }
}

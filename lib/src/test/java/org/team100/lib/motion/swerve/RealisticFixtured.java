package org.team100.lib.motion.swerve;

import java.io.IOException;

import org.junit.jupiter.api.AfterEach;
import org.team100.lib.testing.Timeless;

/**
 * Uses simulated position sensors, must be used with clock control (e.g.
 * {@link Timeless}).
 */
public class RealisticFixtured {
    protected final RealisticFixture fixture;

    public RealisticFixtured() throws IOException {
        fixture = new RealisticFixture();
    }

    @AfterEach
    void close() {
        fixture.close();
    }
}

package org.team100.lib.commands.swerve.manual;

import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.GlobalVelocityR3Logger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.state.ModelR3;

/**
 * Transform manual input into a field-relative velocity.
 * 
 * The input is a twist, so the output is just scaled.
 */
public class ManualFieldRelativeSpeeds implements FieldRelativeDriver {
    private final SwerveKinodynamics m_swerveKinodynamics;
    // LOGGERS
    private final GlobalVelocityR3Logger m_log_scaled;

    public ManualFieldRelativeSpeeds(LoggerFactory parent, SwerveKinodynamics swerveKinodynamics) {
        LoggerFactory child = parent.type(this);
        m_log_scaled = child.globalVelocityR3Logger(Level.TRACE, "scaled");
        m_swerveKinodynamics = swerveKinodynamics;
    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds.
     */
    @Override
    public GlobalVelocityR3 apply(ModelR3 state, Velocity input) {
        // clip the input to the unit circle
        final Velocity clipped = input.clip(1.0);

        // scale to max in both translation and rotation
        final GlobalVelocityR3 scaled = FieldRelativeDriver.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        m_log_scaled.log(() -> scaled);
        return scaled;
    }

    @Override
    public void reset(ModelR3 p) {
        //
    }
}

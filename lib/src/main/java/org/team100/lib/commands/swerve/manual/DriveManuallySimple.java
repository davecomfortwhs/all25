package org.team100.lib.commands.swerve.manual;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.lib.config.DriverSkill;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.hid.Velocity;
import org.team100.lib.motion.swerve.SwerveDriveSubsystem;
import org.team100.lib.motion.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.state.ModelR3;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Select one of two drivers with a button.
 */
public class DriveManuallySimple extends Command {
    /**
     * While driving manually, pay attention to tags even if they are somewhat far
     * away.
     */
    private static final double HEED_RADIUS_M = 6.0;
    /**
     * Velocity control in control units, [-1,1] on all axes. This needs to be
     * mapped to a feasible velocity control as early as possible.
     */
    private final Supplier<Velocity> m_twistSupplier;
    private final DoubleConsumer m_heedRadiusM;
    private final SwerveDriveSubsystem m_drive;
    private final SwerveLimiter m_limiter;
    private final FieldRelativeDriver m_defaultDriver;
    private final FieldRelativeDriver m_alternativeDriver;
    String currentManualMode = null;
    Supplier<Boolean> m_useAlternative;

    boolean wasUsingAlternative = true;

    public DriveManuallySimple(
            Supplier<Velocity> twistSupplier,
            DoubleConsumer heedRadiusM,
            SwerveDriveSubsystem drive,
            SwerveLimiter limiter,
            FieldRelativeDriver driver,
            FieldRelativeDriver alterativeDriver,
            Supplier<Boolean> useAlteranative) {
        m_twistSupplier = twistSupplier;
        m_heedRadiusM = heedRadiusM;
        m_drive = drive;
        m_limiter = limiter;
        m_defaultDriver = driver;
        m_alternativeDriver = alterativeDriver;
        m_useAlternative = useAlteranative;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_heedRadiusM.accept(HEED_RADIUS_M);
        // make sure the limiter knows what we're doing
        m_limiter.updateSetpoint(m_drive.getVelocity());
        m_defaultDriver.reset(m_drive.getState());
    }

    @Override
    public void execute() {
        ModelR3 state = m_drive.getState();
        if (m_useAlternative.get() != wasUsingAlternative) {
            // switch modes
            m_alternativeDriver.reset(state);
        }
        GlobalVelocityR3 v = desiredVelocity(state, m_twistSupplier.get());
        // scale for driver skill.
        GlobalVelocityR3 scaled = GeometryUtil.scale(v, DriverSkill.level().scale());

        // Apply field-relative limits.
        if (Experiments.instance.enabled(Experiment.UseSetpointGenerator)) {
            scaled = m_limiter.apply(scaled);
        }

        m_drive.setVelocity(scaled);
        wasUsingAlternative = m_useAlternative.get();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    private GlobalVelocityR3 desiredVelocity(ModelR3 state, Velocity input) {
        if (m_useAlternative.get()) {
            return m_alternativeDriver.apply(state, input);
        }
        return m_defaultDriver.apply(state, input);
    }
}

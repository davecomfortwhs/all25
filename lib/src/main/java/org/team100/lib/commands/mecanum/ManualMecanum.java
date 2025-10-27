package org.team100.lib.commands.mecanum;

import java.util.function.Supplier;

import org.team100.lib.commands.swerve.manual.FieldRelativeDriver;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mecanum.MecanumDrive100;
import org.team100.lib.motion.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.tuning.Mutable;
import org.team100.lib.util.EnumChooser;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ManualMecanum extends Command {
    public enum InputShaping {
        NONE,
        CLIP,
        SQUASH
    }

    private final Supplier<Velocity> m_velocity;
    private final Mutable m_maxVX;
    private final Mutable m_maxVY;
    private final Mutable m_maxOmega;
    private final SwerveLimiter m_limiter;
    private final MecanumDrive100 m_drive;
    private final EnumChooser<InputShaping> m_chooser;

    public ManualMecanum(
            LoggerFactory parent,
            Supplier<Velocity> velocity,
            double maxVX,
            double maxVY,
            double maxOmega,
            SwerveLimiter limiter,
            MecanumDrive100 drive) {
        if (maxVY > maxVX)
            throw new IllegalArgumentException();
        LoggerFactory log = parent.type(this);
        m_velocity = velocity;
        m_maxVX = new Mutable(log, "maxVX", maxVX);
        m_maxVY = new Mutable(log, "maxVY", maxVY);
        m_maxOmega = new Mutable(log, "maxOmega", maxOmega);
        m_limiter = limiter;
        m_drive = drive;
        m_chooser = new EnumChooser<>("Input Shaping", InputShaping.NONE);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_limiter.updateSetpoint(m_drive.getState().velocity());
    }

    @Override
    public void execute() {
        Rotation2d poseRotation = m_drive.getState().rotation();
        Velocity input = m_velocity.get();
        // clip the input to the diamond shape
        double y_x = m_maxVY.getAsDouble() / m_maxVX.getAsDouble();
        Velocity clippedOrSquashed = switch (m_chooser.get()) {
            case NONE -> input;
            case CLIP -> input.diamond(1, y_x, poseRotation);
            case SQUASH -> input.squashedDiamond(1, y_x, poseRotation);
        };
        GlobalVelocityR3 scaled = FieldRelativeDriver.scale(
                clippedOrSquashed, m_maxVX.getAsDouble(), m_maxOmega.getAsDouble());
        // Apply field-relative limits.
        if (Experiments.instance.enabled(Experiment.UseSetpointGenerator)) {
            scaled = m_limiter.apply(scaled);
        }
        m_drive.setVelocity(scaled);
    }

}

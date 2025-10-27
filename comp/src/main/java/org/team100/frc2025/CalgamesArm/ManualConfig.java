package org.team100.frc2025.CalgamesArm;

import java.util.function.Supplier;

import org.team100.lib.hid.Velocity;
import org.team100.lib.motion.prr.Config;
import org.team100.lib.motion.prr.JointAccelerations;
import org.team100.lib.motion.prr.JointVelocities;

import edu.wpi.first.wpilibj2.command.Command;

/** Use the operator control to "fly" the arm around in config space. */
public class ManualConfig extends Command {

    private final Supplier<Velocity> m_input;
    private final CalgamesMech m_subsystem;

    private Config m_config;
    private JointVelocities m_prev;

    public ManualConfig(
            Supplier<Velocity> input,
            CalgamesMech subsystem) {
        m_input = input;
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_config = m_subsystem.getConfig();
        m_prev = new JointVelocities(0, 0, 0);
    }

    @Override
    public void execute() {
        // input is [-1, 1]
        Velocity input = m_input.get();
        final double dt = 0.02;
        // control is velocity.
        // velocity in m/s and rad/s
        // we want full scale to be about 0.5 m/s and 0.5 rad/s
        JointVelocities jv = new JointVelocities(
                input.x() * 1.5,
                input.y() * 3,
                input.theta() * 3);
        Config newC = m_config.integrate(jv, dt);

        // impose limits; see CalgamesMech for more limits.
        if (newC.shoulderHeight() < 0 || newC.shoulderHeight() > 1.7) {
            newC = new Config(m_config.shoulderHeight(), newC.shoulderAngle(), newC.wristAngle());
        }
        if (newC.shoulderAngle() < -2 || newC.shoulderAngle() > 2) {
            newC = new Config(newC.shoulderHeight(), m_config.shoulderAngle(), newC.wristAngle());
        }
        if (newC.wristAngle() < -1.5 || newC.wristAngle() > 2.1) {
            newC = new Config(newC.shoulderHeight(), newC.shoulderAngle(), m_config.wristAngle());
        }

        // recompute velocity and accel
        JointVelocities newJv = newC.diff(m_config, dt);
        JointAccelerations ja = newJv.diff(m_prev, dt);

        // m_subsystem.set(newC, newJv, ja);
        m_subsystem.set(newC, newJv, ja);
        m_config = newC;
        m_prev = newJv;
    }
}

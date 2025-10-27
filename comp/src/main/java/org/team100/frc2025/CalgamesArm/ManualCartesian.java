package org.team100.frc2025.CalgamesArm;

import java.util.function.Supplier;

import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.hid.Velocity;
import org.team100.lib.state.ControlR3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Use the operator control to "fly" the arm around in config space. */
public class ManualCartesian extends Command {
    private static final boolean DEBUG = false;

    private final Supplier<Velocity> m_input;
    private final CalgamesMech m_subsystem;

    private Pose2d m_pose;

    public ManualCartesian(
            Supplier<Velocity> input,
            CalgamesMech subsystem) {
        m_input = input;
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_pose = m_subsystem.getState().pose();
    }

    @Override
    public void execute() {
        // input is [-1, 1]
        Velocity input = m_input.get();
        final double dt = 0.02;
        // control is velocity.
        // velocity in m/s and rad/s
        // we want full scale to be about 0.5 m/s and 0.5 rad/s
        GlobalVelocityR3 jv = new GlobalVelocityR3(
                input.x() * 1.5,
                input.y() * 1.5,
                input.theta() * 3);

        double x2 = m_pose.getX() + jv.x() * dt;
        double y2 = m_pose.getY() + jv.y() * dt;
        Rotation2d r2 = m_pose.getRotation().plus(new Rotation2d(jv.theta() * dt));
        m_pose = new Pose2d(x2, y2, r2); // our new goal point

        m_subsystem.set(new ControlR3(m_pose));
        if (DEBUG) {
            System.out.printf("pose %s\n", m_pose);
        }
    }
}

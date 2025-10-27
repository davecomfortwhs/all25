package frc.robot;

import org.team100.lib.profile.timed.JerkLimitedTimedProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveZ extends Command {
    private final LynxArm m_arm;
    private final double m_goal;
    private final JerkLimitedTimedProfile m_profile;
    private final Timer m_timer;

    private double m_start;
    private double m_grip;
    private double m_distance;
    private boolean m_done;

    public MoveZ(LynxArm arm, double goal) {
        m_arm = arm;
        m_goal = goal;
        m_profile = new JerkLimitedTimedProfile(1, 1, 10, true);
        m_timer = new Timer();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_start = m_arm.getPosition().p6().getZ();
        m_grip = m_arm.getGrip();
        m_distance = Math.abs(m_start - m_goal);
        m_profile.init(new Control100(), new Model100(m_distance, 0));
        m_timer.restart();
        m_done = false;
    }

    @Override
    public void execute() {
        m_arm.setGrip(m_grip);
        Control100 c = m_profile.sample(m_timer.get());
        double s = c.x() / m_distance;
        double setpoint = MathUtil.interpolate(m_start, m_goal, s);

        double distance = Math.abs(m_goal - setpoint);
        if (distance < 0.001) {
            m_arm.setHeight(m_goal);
            m_done = true;
            return;
        }
        m_arm.setHeight(setpoint);
    }

    @Override
    public boolean isFinished() {
        return m_done;
    }
}

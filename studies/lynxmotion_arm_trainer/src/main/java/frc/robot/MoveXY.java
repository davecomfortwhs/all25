package frc.robot;

import org.team100.lib.profile.timed.JerkLimitedTimedProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** Move in the XY plane only. */
public class MoveXY extends Command {
    private final LynxArm m_arm;
    private final Translation2d m_goal;
    private final JerkLimitedTimedProfile m_profile;
    private final Timer m_timer;

    private Translation2d m_start;
    private double m_grip;
    private double m_distance;
    private boolean m_done;

    public MoveXY(
            LynxArm arm,
            Translation2d goal) {
        m_arm = arm;
        m_goal = goal;
        m_profile = new JerkLimitedTimedProfile(0.1, 1, 10, true);
        m_timer = new Timer();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_start = m_arm.getPosition().p6().toPose2d().getTranslation();
        m_grip = m_arm.getGrip();

        m_distance = m_start.getDistance(m_goal);
        m_profile.init(new Control100(), new Model100(m_distance, 0));
        m_timer.restart();
        m_done = false;
    }

    @Override
    public void execute() {
        m_arm.setGrip(m_grip);
        Control100 c = m_profile.sample(m_timer.get());
        double s = c.x() / m_distance;
        Translation2d setpoint = m_start.interpolate(m_goal, s);
        double distance = m_goal.getDistance(setpoint);
        if (distance < 0.001) {
            m_arm.setPosition(m_goal);
            m_done = true;
            return;
        }
        m_arm.setPosition(setpoint);
    }

    public boolean done() {
        return m_done;
    }

}

package frc.robot;

import org.team100.lib.indicator.SolidIndicator;
import org.team100.lib.util.RoboRioChannel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private final SolidIndicator m_indicator;
    private final XboxController m_controller;

    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);
        m_indicator = new SolidIndicator(new RoboRioChannel(0), 40);
        m_controller = new XboxController(0);
        m_indicator.state(this::states);
        m_indicator.event(m_controller::getXButton, Color.kAqua);
        m_indicator.event(m_controller::getYButton, Color.kWhite);
    }

    /**
     * Arbitrary logic to determine what color to show.
     * 
     * Some guidance for this logic: it would be good for states to be mutually
     * exclusive, so that the order of evaluation here doesn't matter.
     */
    public Color states() {
        if (m_controller.getAButton()) {
            return Color.kRed;
        } else if (m_controller.getBButton()) {
            return Color.kGreen;
        } else {
            return Color.kBlack;
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}

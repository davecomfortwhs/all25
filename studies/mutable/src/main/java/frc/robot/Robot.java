
package frc.robot;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private final XboxController m_controller;
    private final Alert m_alertInfo;
    private final Alert m_alertWarn;
    private final Alert m_alertErr;
    private final Blarg m_blarg;

    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);
        m_controller = new XboxController(0);
        // Alerts are only rendered by glass if they're in the default "group" which is
        // called "Alerts".
        m_alertInfo = new Alert("Info Alert", AlertType.kInfo);
        m_alertWarn = new Alert("Warning Alert", AlertType.kWarning);
        m_alertErr = new Alert("Error Alert", AlertType.kError);

        LoggerFactory root = Logging.instance().rootLogger;
        m_blarg = new Blarg(root);
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_alertInfo.set(m_controller.getAButton());
        m_alertWarn.set(m_controller.getBButton());
        m_alertErr.set(m_controller.getXButton());
        m_alertErr.setText(m_controller.getYButton() ? "Error foo" : "Error bar");

    }

    @Override
    public void teleopInit() {
        System.out.printf("bar %6.3f\n", m_blarg.get());
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

}

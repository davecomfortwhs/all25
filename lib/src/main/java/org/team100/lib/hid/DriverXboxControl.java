package org.team100.lib.hid;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This is a Microsoft Xbox controller, Logitech F310, or similar.
 * 
 * This class exists to group inputs for velocity, and to provide nicer method
 * names.
 * 
 * Do not use stick buttons, they are prone to stray clicks
 */
public class DriverXboxControl {
    private final XboxController m_controller;

    public DriverXboxControl(int port) {
        m_controller = new XboxController(port);
    }

    /**
     * "RC mode 2" control:
     * * right Y (axis 5) is the field "X" direction, ahead
     * * right X (axis 4) is the field "Y" direction, to the left
     * * left X (axis 0) is rotation, counterclockwise
     */
    public Velocity velocity() {
        return ControlUtil.velocity(
                m_controller::getRightY,
                m_controller::getRightX,
                m_controller::getLeftX,
                0.1,
                0.65);
    }

    /** Axis 5 */
    public double rightY() {
        return m_controller.getRightY();
    }

    /** Axis 4 */
    public double rightX() {
        return m_controller.getRightX();
    }

    /** Axis 0 */
    public double leftX() {
        return m_controller.getLeftX();
    }

    public Rotation2d pov() {
        return ControlUtil.pov(m_controller::getPOV);
    }

    public boolean back() {
        return m_controller.getBackButton();
    }

    public boolean start() {
        return m_controller.getStartButton();
    }

    /** Left trigger is all the way in */
    public boolean leftTrigger() {
        return m_controller.getLeftTriggerAxis() > 0.9;
    }

    /** Right trigger is all the way in */
    public boolean rightTrigger() {
        return m_controller.getRightTriggerAxis() > 0.9;
    }

    public boolean leftBumper() {
        return m_controller.getLeftBumperButton();
    }

    public boolean rightBumper() {
        return m_controller.getRightBumperButton();
    }

    /** Button 1 */
    public boolean a() {
        return m_controller.getAButton();
    }

    /** Button 2 */
    public boolean b() {
        return m_controller.getBButton();
    }

    /** Button 3 */
    public boolean x() {
        return m_controller.getXButton();
    }

    /** Button 4 */
    public boolean y() {
        return m_controller.getYButton();
    }

    /** Axis 1 */
    public double leftY() {
        return m_controller.getLeftY();
    }

}

package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * The RC joystick thing joel made.
 * X, Y, and twist should work.
 * POV rotation should work.
 * Only one joystick is required.
 * Operator features are not implemented.
 * Command buttons are not implemented.
 */
public class Pilot {
    private static final double DEADBAND = 0.02;
    private static final double EXPO = 0.5;

    private final GenericHID m_controller;

    public Pilot() {
        m_controller = new GenericHID(0);
    }
    
    public boolean resetRotation0() {
        return button(2);
    }

    public boolean resetRotation180() {
        return button(3);
    }

    /**
     * Applies expo to each axis individually, works for "square" joysticks.
     * The square response of this joystick should be clamped by the consumer.
     */
    public Velocity velocity() {
        double dx = expo(deadband(-1.0 * clamp(axis(1), 1), DEADBAND, 1), EXPO);
        double dy = expo(deadband(-1.0 * clamp(axis(0), 1), DEADBAND, 1), EXPO);
        double dtheta = 0; // there is no rotational velocity control.
        return new Velocity(dx, dy, dtheta);
    }

    public Rotation2d desiredRotation() {
        // the control goes from -1 to 1 in one turn
        double rotControl = m_controller.getRawAxis(5);
        return Rotation2d.fromRotations(rotControl / 2);
    }

    private double axis(int axis) {
        return m_controller.getRawAxis(axis);
    }

    private boolean button(int button) {
        return m_controller.getRawButton(button);
    }
}

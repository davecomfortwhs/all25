package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * The RealFlight USB controller is a basic RC-style control that comes with the
 * RealFlight simulator.
 * 
 * HARDWARE
 * 
 * The control layout is as follows:
 * 
 * left x: axis 4
 * left y: axis 2 (with detents)
 * right x: axis 0
 * right y: axis 1
 * remember buttons are one-based
 * "reset/cancel": button 1
 * left switch: button 2
 * right switch: button 3
 * "menu/select": button 4
 * name: "GP Controller"
 * 
 * MAPPINGS
 * 
 * Right x and y are mapped to cartesian control.
 * Left x is rotational speed
 * Left y is throttle, slow-med-fast.
 * Reset is "reset pose"
 * Oscillate is "menu/select"
 * Left switch is medium speed.
 * Right switch is slow speed.
 */
public class RealFlight {
    private static final double DEADBAND = 0.02;
    private static final double EXPO = 0.5;
    private static final double MEDIUM = 0.5;
    private static final double SLOW = 0.15;

    private final GenericHID hid;

    public RealFlight() {
        hid = new GenericHID(0);
    }

    /**
     * Applies expo to each axis individually, works for "square" joysticks.
     * The square response of this joystick should be clamped by the consumer.
     */

    public Velocity velocity() {
        final double dx = expo(deadband(-1.0 * clamp(scaled(1), 1), DEADBAND, 1), EXPO);
        final double dy = expo(deadband(-1.0 * clamp(scaled(0), 1), DEADBAND, 1), EXPO);
        final double dtheta = expo(deadband(-1.0 * clamp(scaled(4), 1), DEADBAND, 1), EXPO);

        // left = SLOW
        if (hid.getRawButton(2))
            return new Velocity(SLOW * dx, SLOW * dy, SLOW * dtheta);
        // right = MEDIUM
        if (hid.getRawButton(3))
            return new Velocity(MEDIUM * dx, MEDIUM * dy, MEDIUM * dtheta);
        return new Velocity(dx, dy, dtheta);
    }

    public boolean resetRotation0() {
        return hid.getRawButton(1);
    }

    /////////////////////////////////////////

    /**
     * Scale to [-1,1] with the center at 0.
     * Each axis calibration is different.
     */
    private double scaled(int axis) {
        double raw = hid.getRawAxis(axis);
        double zeroed = 0;
        switch (axis) {
            case 0:
                zeroed = raw - 0.043;
                if (zeroed < 0)
                    return zeroed / 0.729;
                return zeroed / 0.784;
            case 1:
                zeroed = raw - 0.169;
                if (zeroed < 0)
                    return zeroed / 0.628;
                return zeroed / 0.619;
            case 2:
                zeroed = raw - 0.137;
                if (zeroed < 0)
                    return zeroed / 0.604;
                return zeroed / 0.643;

            case 3:
                return 0;
            case 4:
                zeroed = raw - 0.075;
                if (zeroed < 0)
                    return zeroed / 0.738;
                return zeroed / 0.776;
            default:
                return 0;
        }
    }
}

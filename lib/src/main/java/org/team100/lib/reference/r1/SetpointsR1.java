package org.team100.lib.reference.r1;

import org.team100.lib.state.Control100;

/**
 * Current and next setpoint.
 * 
 * We keep both so that feedback implementations can choose what to do. Simple
 * feedback can just compare the current setpoint to the current measurement.
 * More clever feedback might extrapolate the current measurements and compare
 * to the next setpoint.
 * 
 * @param current
 * @param next
 */
public record SetpointsR1(Control100 current, Control100 next) {

}

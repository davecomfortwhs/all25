package org.team100.lib.commands.swerve.manual;

import org.team100.lib.hid.Velocity;
import org.team100.lib.state.ModelR3;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface ChassisSpeedDriver  {

    ChassisSpeeds apply(ModelR3 state, Velocity input);

    void reset(ModelR3 state);
}

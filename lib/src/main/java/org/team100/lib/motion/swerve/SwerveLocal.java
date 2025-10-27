package org.team100.lib.motion.swerve;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ChassisSpeedsLogger;
import org.team100.lib.logging.LoggerFactory.SwerveModulePositionsLogger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.module.SwerveModuleCollection;
import org.team100.lib.motion.swerve.module.state.SwerveModulePositions;
import org.team100.lib.motion.swerve.module.state.SwerveModuleStates;
import org.team100.lib.music.Player;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * The swerve drive in local, or robot, reference frame. This class knows
 * nothing about the outside world, it just accepts chassis speeds.
 * 
 * Most methods in this class should be package-private, they're only used by
 * SwerveDriveSubsystem, and by tests.
 */
public class SwerveLocal implements Player {
    private static final boolean DEBUG = false;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final SwerveModuleCollection m_modules;

    private final SwerveModulePositionsLogger m_logPositions;
    private final ChassisSpeedsLogger m_log_chassis_speed;

    public SwerveLocal(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            SwerveModuleCollection modules) {
        LoggerFactory child = parent.type(this);
        m_log_chassis_speed = child.chassisSpeedsLogger(Level.TRACE, "chassis speed");
        m_logPositions = child.swerveModulePositionsLogger(Level.TRACE, "positions");
        m_swerveKinodynamics = swerveKinodynamics;
        m_modules = modules;
    }

    @Override
    public void play(double freq) {
        m_modules.play(freq);
    }

    //////////////////////////////////////////////////////////
    //
    // Actuators. These are mutually exclusive within an iteration.
    //

    /**
     * Discretizes the speeds, calculates the inverse kinematic module states, and
     * sets the module states.
     */
    void setChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleStates states = m_swerveKinodynamics.toSwerveModuleStates(speeds);
        setModuleStates(states);
        m_log_chassis_speed.log(() -> speeds);
    }

    /**
     * Sets the wheels to make an "X" pattern.
     */
    void defense() {
        // not optimizing makes it easier to test, not sure it's worth the slowness.
        setRawModuleStates(SwerveModuleStates.statesX);
    }

    /**
     * Sets wheel rotation to zero, for optimizing steering control.
     */
    void steer0() {
        setRawModuleStates(SwerveModuleStates.states0);
    }

    /**
     * Sets wheel rotation to 90 degrees, for optimizing steering control.
     */
    void steer90() {
        setRawModuleStates(SwerveModuleStates.states90);
    }

    void stop() {
        m_modules.stop();
    }

    /**
     * Set the module states without desaturating.
     * 
     * Works fine with empty angles.
     * 
     * This "raw" mode is just for testing.
     */
    void setRawModuleStates(SwerveModuleStates targetModuleStates) {
        m_modules.setRawDesiredStates(targetModuleStates);
    }

    ////////////////////////////////////////////////////////////////////
    //
    // Observers
    //

    /** Uses Cache so the position is fresh and coherent. */
    SwerveModulePositions positions() {
        return m_modules.positions();
    }

    Translation2d[] getModuleLocations() {
        return m_swerveKinodynamics.getKinematics().getModuleLocations();
    }

    boolean[] atSetpoint() {
        return m_modules.atSetpoint();
    }

    ///////////////////////////////////////////

    void close() {
        m_modules.close();
    }

    void reset() {
        if (DEBUG)
            System.out.println("WARNING: make sure resetting in SwerveLocal doesn't break anything");
        m_modules.reset();
    }

    /** Updates visualization. */
    void periodic() {
        m_logPositions.log(this::positions);
        m_modules.periodic();
    }

    /////////////////////////////////////////////////////////

    private void setModuleStates(SwerveModuleStates states) {
        m_modules.setDesiredStates(states);
    }
}

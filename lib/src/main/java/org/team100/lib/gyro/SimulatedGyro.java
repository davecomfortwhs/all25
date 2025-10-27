package org.team100.lib.gyro;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.DoubleCache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Rotation2dLogger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.module.SwerveModuleCollection;
import org.team100.lib.motion.swerve.module.state.SwerveModuleStates;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A simulated gyro that uses drivetrain odometry.
 */
public class SimulatedGyro implements Gyro {
    private double m_heading = 0;
    private final Rotation2dLogger m_log_yaw;
    private final DoubleLogger m_log_yaw_rate;
    private final SwerveKinodynamics m_kinodynamics;
    private final SwerveModuleCollection m_moduleCollection;

    private final DoubleCache m_headingCache;

    private double m_time = Takt.get();

    public SimulatedGyro(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            SwerveModuleCollection collection) {
        LoggerFactory child = parent.type(this);
        m_log_yaw = child.rotation2dLogger(Level.TRACE, "Yaw NWU (rad)");
        m_log_yaw_rate = child.doubleLogger(Level.TRACE, "Yaw Rate NWU (rad_s)");
        m_kinodynamics = kinodynamics;
        m_moduleCollection = collection;
        m_headingCache = Cache.ofDouble(() -> {
            double dt = dt();
            if (dt > 0.04) {
                // clock is unreliable, ignore it
                dt = 0;
            }
            SwerveModuleStates states = m_moduleCollection.states();
            ChassisSpeeds speeds = m_kinodynamics.toChassisSpeedsWithDiscretization(states, 0.02);
            m_heading += speeds.omegaRadiansPerSecond * dt;
            return m_heading;
        });
    }

    double dt() {
        double now = Takt.get();
        double dt = now - m_time;
        m_time = now;
        return dt;
    }

    @Override
    public Rotation2d getYawNWU() {
        final Rotation2d yawNWU = new Rotation2d(m_headingCache.getAsDouble());
        m_log_yaw.log(() -> yawNWU);
        return yawNWU;
    }

    @Override
    public double getYawRateNWU() {
        SwerveModuleStates states = m_moduleCollection.states();
        ChassisSpeeds speeds = m_kinodynamics.toChassisSpeedsWithDiscretization(states, 0.02);
        double yawRateRad_S = speeds.omegaRadiansPerSecond;
        m_log_yaw_rate.log(() -> yawRateRad_S);
        return yawRateRad_S;
    }

    @Override
    public Rotation2d getPitchNWU() {
        return Rotation2d.kZero;
    }

    @Override
    public Rotation2d getRollNWU() {
        return Rotation2d.kZero;
    }

    @Override
    public void periodic() {
        //
    }

}

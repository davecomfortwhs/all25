package org.team100.lib.motion.swerve.module;

import org.team100.lib.config.Identity;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.module.WCPSwerveModule100.DriveRatio;
import org.team100.lib.motion.swerve.module.state.SwerveModulePositions;
import org.team100.lib.motion.swerve.module.state.SwerveModuleStates;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.music.Player;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;

/**
 * Represents the modules in the drivetrain.
 * Do not put logic here; this is just for bundling the modules together.
 */
public class SwerveModuleCollection implements Player {
    private static final boolean DEBUG = false;
    private static final String SWERVE_MODULES = "Swerve Modules";
    private static final String FRONT_LEFT = "Front Left";
    private static final String FRONT_RIGHT = "Front Right";
    private static final String REAR_LEFT = "Rear Left";
    private static final String REAR_RIGHT = "Rear Right";

    private final SwerveModule100 m_frontLeft;
    private final SwerveModule100 m_frontRight;
    private final SwerveModule100 m_rearLeft;
    private final SwerveModule100 m_rearRight;

    SwerveModuleCollection(
            SwerveModule100 frontLeft,
            SwerveModule100 frontRight,
            SwerveModule100 rearLeft,
            SwerveModule100 rearRight) {
        m_frontLeft = frontLeft;
        m_frontRight = frontRight;
        m_rearLeft = rearLeft;
        m_rearRight = rearRight;
    }

    /**
     * Creates collections according to Identity.
     */
    public static SwerveModuleCollection get(
            LoggerFactory parent,
            double supplyLimit,
            double statorLimit,
            SwerveKinodynamics kinodynamics) {
        LoggerFactory collectionLogger = parent.name(SWERVE_MODULES);
        LoggerFactory frontLeftLogger = collectionLogger.name(FRONT_LEFT);
        LoggerFactory frontRightLogger = collectionLogger.name(FRONT_RIGHT);
        LoggerFactory rearLeftLogger = collectionLogger.name(REAR_LEFT);
        LoggerFactory rearRightLogger = collectionLogger.name(REAR_RIGHT);

        switch (Identity.instance) {
            case COMP_BOT:
                System.out.println("************** WCP MODULES w/Duty-Cycle Encoders **************");
                return new SwerveModuleCollection(
                        WCPSwerveModule100.getKrakenDrive(frontLeftLogger, supplyLimit, statorLimit,
                                new CanId(2),
                                DriveRatio.MEDIUM,
                                new CanId(1),
                                new RoboRioChannel(6),
                                0.893686,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode.COAST, MotorPhase.REVERSE),
                        WCPSwerveModule100.getKrakenDrive(frontRightLogger, supplyLimit, statorLimit,
                                new CanId(4),
                                DriveRatio.MEDIUM,
                                new CanId(3),
                                new RoboRioChannel(7),
                                0.976568,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode.COAST, MotorPhase.REVERSE),
                        WCPSwerveModule100.getKrakenDrive(rearLeftLogger, supplyLimit, statorLimit,
                                new CanId(8),
                                DriveRatio.MEDIUM,
                                new CanId(7),
                                new RoboRioChannel(9),
                                0.312068,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode.COAST, MotorPhase.REVERSE),
                        WCPSwerveModule100.getKrakenDrive(rearRightLogger, supplyLimit, statorLimit,
                                new CanId(6),
                                DriveRatio.MEDIUM,
                                new CanId(5),
                                new RoboRioChannel(8),
                                0.842786,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode.COAST, MotorPhase.REVERSE));
            case SWERVE_ONE:
                System.out.println("************** WCP MODULES w/Duty-Cycle Encoders **************");
                return new SwerveModuleCollection(
                        WCPSwerveModule100.getFalconDrive(frontLeftLogger, supplyLimit, statorLimit,
                                new CanId(12), // drive
                                DriveRatio.FAST,
                                new CanId(32), // steer
                                new RoboRioChannel(6),
                                0.160218,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode.COAST, MotorPhase.REVERSE),
                        WCPSwerveModule100.getFalconDrive(frontRightLogger, supplyLimit, statorLimit,
                                new CanId(11), // drive
                                DriveRatio.FAST,
                                new CanId(30), // steer
                                new RoboRioChannel(8),
                                0.876519,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode.COAST, MotorPhase.REVERSE),
                        WCPSwerveModule100.getFalconDrive(rearLeftLogger, supplyLimit, statorLimit,
                                new CanId(21), // drive
                                DriveRatio.FAST,
                                new CanId(31), // steer
                                new RoboRioChannel(7),
                                0.406423,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode.COAST, MotorPhase.REVERSE),
                        WCPSwerveModule100.getFalconDrive(rearRightLogger, supplyLimit, statorLimit,
                                new CanId(22), // drive
                                DriveRatio.FAST,
                                new CanId(33), // steer
                                new RoboRioChannel(9),
                                0.032502,
                                kinodynamics,
                                EncoderDrive.INVERSE, NeutralMode.COAST, MotorPhase.REVERSE));
            case BETA_BOT:
            case SWERVE_TWO:
            case BLANK:
            default:
                if (DEBUG)
                    System.out.println("************** SIMULATED MODULES **************");
                /*
                 * Uses simulated position sensors, must be used with clock control (e.g.
                 * {@link Timeless}).
                 */
                return new SwerveModuleCollection(
                        SimulatedSwerveModule100.get(frontLeftLogger, kinodynamics),
                        SimulatedSwerveModule100.get(frontRightLogger, kinodynamics),
                        SimulatedSwerveModule100.get(rearLeftLogger, kinodynamics),
                        SimulatedSwerveModule100.get(rearRightLogger, kinodynamics));
        }
    }

    public static SwerveModuleCollection forTest(LoggerFactory log, SwerveKinodynamics kinodynamics) {
        return new SwerveModuleCollection(
                SimulatedSwerveModule100.withInstantaneousSteering(log, kinodynamics),
                SimulatedSwerveModule100.withInstantaneousSteering(log, kinodynamics),
                SimulatedSwerveModule100.withInstantaneousSteering(log, kinodynamics),
                SimulatedSwerveModule100.withInstantaneousSteering(log, kinodynamics));
    }

    //////////////////////////////////////////////////
    //
    // Actuators
    //

    /**
     * Optimizes.
     * 
     * Works fine with empty angles.
     * 
     * @param swerveModuleStates
     */
    public void setDesiredStates(SwerveModuleStates swerveModuleStates) {
        if (DEBUG) {
            System.out.printf("setDesiredStates() %s\n", swerveModuleStates);
        }
        m_frontLeft.setDesiredState(swerveModuleStates.frontLeft());
        m_frontRight.setDesiredState(swerveModuleStates.frontRight());
        m_rearLeft.setDesiredState(swerveModuleStates.rearLeft());
        m_rearRight.setDesiredState(swerveModuleStates.rearRight());
    }

    /**
     * Does not optimize.
     * 
     * This "raw" mode is just for testing.
     * 
     * Works fine with empty angles.
     */
    public void setRawDesiredStates(SwerveModuleStates swerveModuleStates) {
        m_frontLeft.setRawDesiredState(swerveModuleStates.frontLeft());
        m_frontRight.setRawDesiredState(swerveModuleStates.frontRight());
        m_rearLeft.setRawDesiredState(swerveModuleStates.rearLeft());
        m_rearRight.setRawDesiredState(swerveModuleStates.rearRight());
    }

    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_rearLeft.stop();
        m_rearRight.stop();
    }

    public void reset() {
        m_frontLeft.reset();
        m_frontRight.reset();
        m_rearLeft.reset();
        m_rearRight.reset();
    }

    //////////////////////////////////////////////////////
    //
    // Observers
    //

    /** Uses Cache so the positions are fresh and coherent. */
    public SwerveModulePositions positions() {
        return new SwerveModulePositions(
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition());
    }

    public double[] turningPosition() {
        return new double[] {
                m_frontLeft.turningPosition(),
                m_frontRight.turningPosition(),
                m_rearLeft.turningPosition(),
                m_rearRight.turningPosition()
        };
    }

    /** FOR TEST ONLY */
    public SwerveModuleStates states() {
        return new SwerveModuleStates(
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState());
    }

    public boolean[] atSetpoint() {
        return new boolean[] {
                m_frontLeft.atSetpoint(),
                m_frontRight.atSetpoint(),
                m_rearLeft.atSetpoint(),
                m_rearRight.atSetpoint()
        };
    }

    ////////////////////////////////////////////

    public void close() {
        m_frontLeft.close();
        m_frontRight.close();
        m_rearLeft.close();
        m_rearRight.close();
    }

    public SwerveModule100[] modules() {
        return new SwerveModule100[] {
                m_frontLeft,
                m_frontRight,
                m_rearLeft,
                m_rearRight };
    }

    /** Updates visualization. */
    public void periodic() {
        m_frontLeft.periodic();
        m_frontRight.periodic();
        m_rearLeft.periodic();
        m_rearRight.periodic();
    }

    @Override
    public void play(double freq) {
        m_frontLeft.play(freq);
        m_frontRight.play(freq);
        m_rearLeft.play(freq);
        m_rearRight.play(freq);
    }
}

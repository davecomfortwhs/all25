package org.team100.frc2025.grip;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.ctre.Talon6Encoder;
import org.team100.lib.encoder.sim.SimulatedBareEncoder;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.motor.ctre.Kraken6Motor;
import org.team100.lib.motor.sim.LazySimulatedBareMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.music.Music;
import org.team100.lib.sensor.LaserCan100;
import org.team100.lib.util.CanId;

import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase implements Music {

    private final BooleanLogger coralLogger;

    private static final int NEAR = 50;
    private final BareMotor m_algaeMotor;
    private final LinearMechanism m_leftMech;
    private final LinearMechanism m_rightMech;
    private final LinearMechanism m_algaeMech;
    private final LaserCan100 m_rightLaser;
    @SuppressWarnings("unused")
    private final LaserCan100 m_frontLaser;
    private final LaserCan100 m_backLaser;
    private final LaserCan100 m_leftLaser;

    public Manipulator(LoggerFactory parent) {
        LoggerFactory log = parent.name("Manipulator");

        LoggerFactory algaeMotorLog = log.name("Algae");
        LoggerFactory leftMotorLog = log.name("leftMotor");
        LoggerFactory rightMotorLog = log.name("rightMotor");
        coralLogger = log.booleanLogger(Level.TRACE, "Coral Detection");
        switch (Identity.instance) {
            case COMP_BOT -> {
                // Set specific parameters for the competition robot
                Kraken6Motor leftMotor = new Kraken6Motor(leftMotorLog, new CanId(19), NeutralMode.COAST,
                        MotorPhase.FORWARD,
                        40, // og 40
                        40, // og 40
                        PIDConstants.zero(leftMotorLog),
                        Feedforward100.makeShooterFalcon6(leftMotorLog));
                Kraken6Motor rightMotor = new Kraken6Motor(rightMotorLog, new CanId(20), NeutralMode.COAST,
                        MotorPhase.REVERSE,
                        40, // og 40
                        40, // og 40
                        PIDConstants.zero(rightMotorLog),
                        Feedforward100.makeShooterFalcon6(rightMotorLog));
                Kraken6Motor algaeMotor = new Kraken6Motor(algaeMotorLog, new CanId(21), NeutralMode.COAST,
                        MotorPhase.FORWARD,
                        120, // og 120
                        120, // og 120
                        PIDConstants.zero(algaeMotorLog),
                        Feedforward100.makeShooterFalcon6(algaeMotorLog));
                algaeMotor.setTorqueLimit(4);
                m_algaeMotor = algaeMotor;
                m_rightLaser = new LaserCan100(new CanId(17));
                m_frontLaser = new LaserCan100(new CanId(16));
                m_backLaser = new LaserCan100(new CanId(18));
                m_leftLaser = new LaserCan100(new CanId(15));
                m_leftMech = new LinearMechanism(leftMotorLog, leftMotor, new Talon6Encoder(log, leftMotor),
                        16, 0.1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_rightMech = new LinearMechanism(rightMotorLog, rightMotor, new Talon6Encoder(log, rightMotor),
                        16, 0.1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_algaeMech = new LinearMechanism(algaeMotorLog, algaeMotor, new Talon6Encoder(log, algaeMotor),
                        16, 0.1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
            }
            default -> {
                BareMotor leftMotor = new SimulatedBareMotor(log, 600);
                SimulatedBareEncoder leftEncoder = new SimulatedBareEncoder(log, leftMotor);
                BareMotor rightMotor = new SimulatedBareMotor(log, 600);
                SimulatedBareEncoder rightEncoder = new SimulatedBareEncoder(log, rightMotor);
                // simulated algae motor gets overloaded 2 sec after starting
                BareMotor algaeMotor = new LazySimulatedBareMotor(
                        new SimulatedBareMotor(log, 600), 2);
                SimulatedBareEncoder algaeEncoder = new SimulatedBareEncoder(log, algaeMotor);
                m_algaeMotor = algaeMotor;
                m_leftMech = new LinearMechanism(log, leftMotor, leftEncoder,
                        1, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_rightMech = new LinearMechanism(log, rightMotor, rightEncoder,
                        1, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_algaeMech = new LinearMechanism(log, algaeMotor, algaeEncoder,
                        1, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_rightLaser = new LaserCan100();
                m_frontLaser = new LaserCan100();
                m_backLaser = new LaserCan100();
                m_leftLaser = new LaserCan100();
            }
        }
    }

    @Override
    public Command play(double freq) {
        return run(() -> {
            m_leftMech.play(freq);
            m_rightMech.play(freq);
            m_algaeMech.play(freq);
        });
    }

    /** Intake and hold. */
    public void intakeCenter() {
        if (hasCoral()) {
            stopMotors();
        } else if (hasCoralSideways()) {
            m_algaeMech.setDutyCycle(-1);
            m_leftMech.setDutyCycle(0.3);
            m_rightMech.setDutyCycle(-0.3);
        } else {
            m_algaeMech.setDutyCycle(-1);
            m_leftMech.setDutyCycle(1);
            m_rightMech.setDutyCycle(0.5);
        }
    }

    public boolean hasCoral() {
        if (Identity.instance.equals(Identity.BLANK))
            return false;
        return coralIsClose(m_backLaser);
    }

    public void ejectCenter() {
        m_algaeMech.setDutyCycle(1);
        m_leftMech.setDutyCycle(-0.75);
        m_rightMech.setDutyCycle(-0.75);
    }

    public void ejectCenterBack() {
        m_algaeMech.setDutyCycle(-1);
        m_leftMech.setDutyCycle(.75);
        m_rightMech.setDutyCycle(.75);
    }

    public void intakeSideways() {
        if (hasCoralSideways()) {
            stopMotors();
            m_algaeMech.setDutyCycle(-1);
        } else {
            m_algaeMech.setDutyCycle(-1);
            if (coralIsClose(m_leftLaser)) {
                m_leftMech.setDutyCycle(0.5);
                m_rightMech.setDutyCycle(-0.5);
            } else {
                m_leftMech.setDutyCycle(-0.5);
                m_rightMech.setDutyCycle(0.5);
            }
        }
    }

    public boolean intakingCoral() {
        return Math.abs(m_leftMech.getVelocityM_S()) > 0;
    }

    public boolean intakingAlgae() {
        return Math.abs(m_algaeMech.getVelocityM_S()) > 0;
    }

    public boolean hasCoralSideways() {
        return coralIsClose(m_leftLaser) && coralIsClose(m_rightLaser);
    }

    public void stopMotors() {
        m_algaeMech.setDutyCycle(0);
        m_leftMech.setDutyCycle(0);
        m_rightMech.setDutyCycle(0);
    }

    /**
     * Current is high when the algae is in.
     * (...and also at startup so include a delay.)
     */
    public boolean hasAlgae() {
        return m_algaeMotor.getCurrent() > 50;
    }

    /////////////////////////////////////////////////
    //
    // COMMANDS

    /** This is not "hold position" it is "disable". */
    public Command stop() {
        return startRun(this::lowAlgaeTorque, this::stopMotors);
    }

    public Command algaeHold() {
        return startRun(this::lowAlgaeTorque, this::intakeAlgae);
    }

    public Command algaeIntake() {
        return startRun(this::highAlgaeTorque, this::intakeAlgae);
    }

    public Command algaeEject() {
        return startRun(this::highAlgaeTorque, this::ejectAlgae);
    }

    /** Intake and hold. */
    public Command centerIntake() {
        return startRun(this::highAlgaeTorque, this::intakeCenter);
    }

    public Command sidewaysIntake() {
        return startRun(this::highAlgaeTorque, this::intakeSideways);
    }

    public Command sidewaysHold() {
        return startRun(this::lowCoralTorque, this::intakeSideways);
    }

    public Command centerEject() {
        return run(this::ejectCenter);
    }

    public Command centerEjectBack() {
        return run(this::ejectCenterBack);
    }

    //////////////////////////////////////////////////

    /**
     * Set high current limits.
     * Previous grip used 90A stator current, with a motor with kT of 0.018 Nm/amp,
     * so 1.62 Nm.
     */
    private void highAlgaeTorque() {
        m_algaeMotor.setTorqueLimit(3);
    }

    /**
     * Set moderate current limits.
     * Previous grip used 35A stator current, with a motor with kT of 0.018 Nm/amp,
     * so 0.63 Nm.
     */
    private void lowAlgaeTorque() {
        m_algaeMotor.setTorqueLimit(2.5);
    }

    private void lowCoralTorque() {
        m_algaeMotor.setTorqueLimit(.6);
    }

    private void intakeAlgae() {
        m_algaeMech.setDutyCycle(1);
    }

    public void ejectAlgae() {
        m_algaeMech.setDutyCycle(-1);
    }

    @Override
    public void periodic() {
        m_rightMech.periodic();
        m_leftMech.periodic();
        m_algaeMech.periodic();
        m_algaeMotor.periodic();
        coralLogger.log(this::hasCoral);
    }
    ///////////////////////////////////////////////

    private static boolean coralIsClose(LaserCan100 sensor) {
        Measurement m = sensor.getMeasurement();
        if (m == null)
            return false;
        return m.distance_mm < NEAR;
    }
}

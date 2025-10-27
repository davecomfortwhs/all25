package org.team100.lib.motor;

import java.util.function.Consumer;
import java.util.function.ToDoubleFunction;

/** Treat a group of motors as a single motor. */
public class BareMotorGroup implements BareMotor {
    BareMotor[] m_motors;

    public BareMotorGroup(BareMotor... motors) {
        m_motors = motors;
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        apply((m) -> m.setTorqueLimit(torqueNm));
    }

    @Override
    public void setDutyCycle(double output) {
        apply((m) -> m.setDutyCycle(output));
    }

    @Override
    public void setVelocity(double velocityRad_S, double accelRad_S2, double torqueNm) {
        apply((m) -> m.setVelocity(velocityRad_S, accelRad_S2, torqueNm));
    }

    @Override
    public double getVelocityRad_S() {
        return mean((m) -> m.getVelocityRad_S());
    }

    @Override
    public double getUnwrappedPositionRad() {
        return mean((m) -> m.getUnwrappedPositionRad());
    }

    @Override
    public double getCurrent() {
        return mean((m) -> m.getCurrent());
    }

    @Override
    public void setUnwrappedEncoderPositionRad(double positionRad) {
        apply((m) -> m.setUnwrappedEncoderPositionRad(positionRad));
    }

    @Override
    public void setUnwrappedPosition(double positionRad, double velocityRad_S, double accelRad_S2, double torqueNm) {
        apply((m) -> m.setUnwrappedPosition(positionRad, velocityRad_S, accelRad_S2, torqueNm));
    }

    @Override
    public double kROhms() {
        return mean((m) -> m.kROhms());
    }

    @Override
    public double kTNm_amp() {
        return mean((m) -> m.kTNm_amp());
    }

    @Override
    public void stop() {
        apply((m) -> m.stop());
    }

    @Override
    public void reset() {
        apply((m) -> m.reset());
    }

    @Override
    public void close() {
        apply((m) -> m.close());
    }

    @Override
    public void periodic() {
        apply((m) -> m.periodic());
    }

    @Override
    public void play(double freq) {
        apply((m) -> m.play(freq));
    }

    private void apply(Consumer<BareMotor> f) {
        for (BareMotor m : m_motors) {
            f.accept(m);
        }
    }

    private double mean(ToDoubleFunction<BareMotor> f) {
        double v = 0;
        for (BareMotor m : m_motors) {
            v += f.applyAsDouble(m);
        }
        v /= m_motors.length;
        return v;
    }

}

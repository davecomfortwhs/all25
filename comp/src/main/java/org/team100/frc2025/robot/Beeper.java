package org.team100.frc2025.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;

/** Plays warnings using the CTRE music feature. */
@SuppressWarnings("unused")
public class Beeper {
    private static final double BPM = 120;
    private static final double A5 = 880;
    private static final double Bflat5 = 932;
    private static final double D5 = 1175;
    private static final double Eflat5 = 1245;
    private static final double F5 = 1397;
    private static final double A6 = 1670;
    private static final double Bflat6 = 1865;

    private final Machinery m_machinery;

    public Beeper(Machinery machinery) {
        m_machinery = machinery;
    }

    public Command play(double freq) {
        return parallel(
                m_machinery.m_mech.play(freq),
                m_machinery.m_manipulator.play(freq),
                m_machinery.m_drive.play(freq));
    }

    /**
     * Three beeps and one long beep, approximately
     * 
     * | q qr q qr | q qr h |
     * 
     * at Allegro tempo.
     */
    public Command start() {
        return sequence(
                quarterNote(Bflat5),
                quarterRest(),
                quarterNote(Bflat5),
                quarterRest(),
                quarterNote(Bflat5),
                quarterRest(),
                halfNote(Bflat6),
                halfRest());
    }

    public Command progress() {
        return sequence(
                quarterNote(Bflat5),
                quarterRest());
    }

    public Command done() {
        return sequence(
                eighthNote(F5),
                eighthNote(D5),
                eighthNote(Eflat5),
                halfNote(F5));
    }

    public Command eighthNote(double freq) {
        return play(freq).withTimeout(eighth());
    }

    public Command quarterNote(double freq) {
        return play(freq).withTimeout(quarter());
    }

    public Command halfNote(double freq) {
        return play(freq).withTimeout(half());
    }

    public Command quarterRest() {
        return play(0).withTimeout(quarter());
    }

    public Command halfRest() {
        return play(0).withTimeout(half());
    }

    private double eighth() {
        return 30 / BPM;
    }

    private double quarter() {
        return 60 / BPM;
    }

    private double half() {
        return 120 / BPM;
    }

}

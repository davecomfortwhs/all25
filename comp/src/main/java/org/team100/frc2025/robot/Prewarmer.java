package org.team100.frc2025.robot;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.coherence.Takt;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.TimingConstraintFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Exercise some code to try to force various initialization and GC delays to
 * happen right now.
 */
public class Prewarmer {
    public static void init(Machinery machinery) {
        System.out.println("\n*** PREWARM START");

        final LoggerFactory logger = Logging.instance().rootLogger.name("Prewarmer");
        double startS = Takt.actual();

        // Exercise the trajectory planner.
        List<HolonomicPose2d> waypoints = new ArrayList<>();
        waypoints.add(new HolonomicPose2d(
                new Translation2d(),
                Rotation2d.kZero,
                Rotation2d.kZero));
        waypoints.add(new HolonomicPose2d(
                new Translation2d(1, 0),
                Rotation2d.kZero,
                Rotation2d.kZero));
        TrajectoryPlanner planner = new TrajectoryPlanner(
                new TimingConstraintFactory(machinery.m_swerveKinodynamics).medium(logger));
        planner.restToRest(waypoints);

        // Exercise the drive motors.
        machinery.m_drive.setVelocity(new GlobalVelocityR3(0, 0, 0));

        // Exercise some mechanism commands.
        Command c = machinery.m_mech.homeToL4();
        c.initialize();
        c.execute();

        c = machinery.m_mech.pickWithProfile();
        c.initialize();
        c.execute();

        machinery.m_mech.stop();

        c = machinery.m_manipulator.centerIntake();
        c.initialize();
        c.execute();

        machinery.m_manipulator.stopMotors();

        c = machinery.m_climber.goToIntakePosition();
        c.initialize();
        c.execute();

        machinery.m_climber.stopMotor();

        c = machinery.m_climberIntake.intake();
        c.initialize();
        c.execute();

        machinery.m_climberIntake.stopMotor();

        // Force full garbage collection.
        // This reduces the allocated heap size, not just the used heap size, which
        // means more-frequent and smaller subsequent GC's.
        System.gc();

        double endS = Takt.actual();

        // the duty cycle encoder produces garbage for a few seconds so sleep.
        try {
            System.out.println("Waiting for DutyCycle sensors to work ...");
            Thread.sleep(1000);
            System.out.println("Waiting for DutyCycle sensors to work ...");
            Thread.sleep(1000);
            System.out.println("Waiting for DutyCycle sensors to work ...");
            Thread.sleep(1000);
            System.out.println("Done!");
        } catch (InterruptedException e) {
        }
        System.out.printf("Total Logger Keys: %d\n", Logging.instance().keyCount());
        System.out.printf("\n*** PREWARM END ET: %f\n", endS - startS);
    }

}

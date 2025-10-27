package org.team100.frc2025;

import java.io.IOException;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.JvmLogger;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.IntLogger;
import org.team100.lib.logging.Logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot100 {
    private static final boolean DEBUG = false;
    private final DoubleLogger m_log_ds_MatchTime;
    private final BooleanLogger m_log_ds_AutonomousEnabled;
    private final BooleanLogger m_log_ds_TeleopEnabled;
    private final BooleanLogger m_log_ds_FMSAttached;
    private final IntLogger m_log_key_list_size;
    private final DoubleLogger m_log_voltage;
    private final JvmLogger m_jvmLogger;
    private final DoubleLogger m_log_update;

    private RobotContainer m_robotContainer;

    public Robot() {
        LoggerFactory dsLog = m_robotLogger.name("DriverStation");
        m_log_ds_MatchTime = dsLog.doubleLogger(Level.TRACE, "MatchTime");
        m_log_ds_AutonomousEnabled = dsLog.booleanLogger(Level.TRACE, "AutonomousEnabled");
        m_log_ds_TeleopEnabled = dsLog.booleanLogger(Level.TRACE, "TeleopEnabled");
        m_log_ds_FMSAttached = dsLog.booleanLogger(Level.TRACE, "FMSAttached");
        m_log_key_list_size = m_robotLogger.intLogger(Level.TRACE, "key list size");
        m_log_voltage = m_robotLogger.doubleLogger(Level.TRACE, "voltage");
        m_jvmLogger = new JvmLogger(m_robotLogger);
        m_log_update = m_robotLogger.doubleLogger(Level.COMP, "update time (s)");
        // 4/2/25 Joel added this to try to avoid doing extra work
        // setNetworkTablesFlushEnabled(false);
        // CanBridge.runTCP();
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // RobotInit is called once by TimedRobot.startCompetition.
    //

    @Override
    public void robotInit() {
        System.out.printf("WPILib Version: %s\n", WPILibVersion.Version); // 2023.2.1
        System.out.printf("RoboRIO serial number: %s\n", RobotController.getSerialNumber());
        System.out.printf("Identity: %s\n", Identity.instance.name());
        RobotController.setBrownoutVoltage(5.5);
        banner();

        // By default, LiveWindow turns off the CommandScheduler in test mode,
        // but we don't want that.
        enableLiveWindowInTest(false);

        // log what the scheduler is doing
        SmartDashboard.putData(CommandScheduler.getInstance());

        try {
            m_robotContainer = new RobotContainer(this);
        } catch (IOException e) {
            throw new IllegalStateException("Robot Container Instantiation Failed", e);
        }

        m_robotContainer.onInit();

        NetworkTableInstance.getDefault().startServer();

        // DataLogManager.start();

        System.out.printf("Total Logger Keys: %d\n", Logging.instance().keyCount());

        // This reduces the allocated heap size, not just the used heap size, which
        // means more-frequent and smaller subsequent GC's.
        System.gc();
    }

    /**
     * robotPeriodic is called in the IterativeRobotBase.loopFunc, which is what the
     * TimedRobot runs in the main loop.
     * 
     * This is what should do all the work.
     */
    @Override
    public void robotPeriodic() {
        try {
            // real-time priority for this thread while it's running robotPeriodic.
            // see
            // https://github.com/Mechanical-Advantage/AdvantageKit/blob/a86d21b27034a36d051798e3eaef167076cd302b/template_projects/sources/vision/src/main/java/frc/robot/Robot.java#L90
            // This seems to interfere with CAN at startup
            // Threads.setCurrentThreadPriority(true, 99);

            // Advance the drumbeat.
            Takt.update();

            // Take all the measurements we can, as soon and quickly as possible.
            updateCaches();

            // Run one iteration of the command scheduler.
            CommandScheduler.getInstance().run();

            // Actuate LEDs, do some logging.
            m_robotContainer.periodic();

            m_log_ds_MatchTime.log(DriverStation::getMatchTime);
            m_log_ds_AutonomousEnabled.log(DriverStation::isAutonomousEnabled);
            m_log_ds_TeleopEnabled.log(DriverStation::isTeleopEnabled);
            m_log_ds_FMSAttached.log(DriverStation::isFMSAttached);

            m_jvmLogger.logGarbageCollectors();
            m_jvmLogger.logMemoryPools();
            m_jvmLogger.logMemoryUsage();

            if (Experiments.instance.enabled(Experiment.FlushOften)) {
                // Util.warn("FLUSHING EVERY LOOP, DO NOT USE IN COMP");
                NetworkTableInstance.getDefault().flush();
            }
        } finally {
            // This thread should have low priority when the main loop isn't running.
            // Threads.setCurrentThreadPriority(false, 0);
        }
    }

    /** Update the measurement caches. */
    private void updateCaches() {
        double startUpdateS = Takt.actual();
        Cache.refresh();
        m_log_update.log(() -> (Takt.actual() - startUpdateS));
    }

    //////////////////////////////////////////////////////////////////////
    //
    // Exits are called first when mode changes
    //
    @Override
    public void testExit() {
        clearCommands();
    }

    //////////////////////////////////////////////////////////////////////
    //
    // Inits are called on mode change, after exiting the previous mode.
    //

    @Override
    public void autonomousInit() {
        m_robotContainer.onAuto();
        m_robotContainer.scheduleAuton();
    }

    @Override
    public void teleopInit() {
        // this cancels all the default commands, resulting in them being rescheduled
        // immediately, which seems like maybe not great?
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testInit() {
        clearCommands();
        for (Topic t : NetworkTableInstance.getDefault().getTopics()) {
            if (DEBUG)
                System.out.printf("%s\n", t.getName());
        }
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // Mode-specific periodics should do nothing, to avoid caching anything.
    //

    @Override
    public void disabledPeriodic() {
        int keyListSize = NetworkTableInstance.getDefault().getTable("Vision").getKeys().size();
        m_log_key_list_size.log(() -> keyListSize);
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
        m_log_voltage.log(RobotController::getBatteryVoltage);
    }

    @Override
    public void testPeriodic() {
    }

    //////////////////////////////////////////////////////////////////////
    //
    // Simulation init is called once right after RobotInit.
    //

    @Override
    public void simulationInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    //////////////////////////////////////////////////////////////////////
    //
    // Simulation periodic is called after everything else; leave it empty.
    //

    @Override
    public void simulationPeriodic() {
        //
    }

    @Override
    public void close() {
        super.close();
        m_robotContainer.close();
    }

    ///////////////////////////////////////////////////////////////////////

    private void clearCommands() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearComposedCommands();
    }

    private void banner() {
        StringBuilder b = new StringBuilder();
        b.append("\n");
        b.append("..########.########....###....##.....##.......##.....#####.....#####....\n");
        b.append(".....##....##.........##.##...###...###.....####....##...##...##...##...\n");
        b.append(".....##....##........##...##..####.####.......##...##.....##.##.....##..\n");
        b.append(".....##....######...##.....##.##.###.##.......##...##.....##.##.....##..\n");
        b.append(".....##....##.......#########.##.....##.......##...##.....##.##.....##..\n");
        b.append(".....##....##.......##.....##.##.....##.......##....##...##...##...##...\n");
        b.append(".....##....########.##.....##.##.....##.....######...#####.....#####....\n");
        b.append("\n");
        System.out.println(b.toString());

    }
}
package org.team100.lib.logging;

import org.team100.lib.logging.primitive.NTPrimitiveLogger;
import org.team100.lib.logging.primitive.PrimitiveLogger;
import org.team100.lib.util.NamedChooser;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Logging singleton.
 * 
 * If you use this logger you'll want to set the log level.
 */
public class Logging {
    private static final Level DEFAULT_LEVEL = Level.TRACE;

    private PrimitiveLogger ntLogger;

    private static final SendableChooser<Level> m_LevelChooser = new NamedChooser<>("Log Level");

    static {
        for (Level level : Level.values()) {
            m_LevelChooser.addOption(level.name(), level);
        }
        m_LevelChooser.setDefaultOption(DEFAULT_LEVEL.name(), DEFAULT_LEVEL);
        SmartDashboard.putData(m_LevelChooser);
    }

    private static final Logging instance = new Logging();

    /**
     * root is "field", with a ".type"->"Field2d" entry as required by glass.
     */
    public final LoggerFactory fieldLogger;
    /** root is "log". */
    public final LoggerFactory rootLogger;

    /**
     * Clients should use the static instance, not the constructor.
     */
    private Logging() {
        ntLogger = new NTPrimitiveLogger();
        fieldLogger = new LoggerFactory(this::getLevel, "field", ntLogger);
        rootLogger = new LoggerFactory(this::getLevel, "log", ntLogger);
        fieldLogger.stringLogger(Level.COMP, ".type").log(() -> "Field2d");

        // turn off the CTRE log we never use
        SignalLogger.enableAutoLogging(false);
    }

    public int keyCount() {
        if (ntLogger != null)
            return ntLogger.keyCount();
        return 0;
    }

    public Level getLevel() {
        return m_LevelChooser.getSelected();
    }

    /** The logging singleton. */
    public static Logging instance() {
        return instance;
    }
}
package org.team100.lib.motion.swerve;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.CotemporalCache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.DriverSkill;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.localization.FreshSwerveEstimate;
import org.team100.lib.localization.OdometryUpdater;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.EnumLogger;
import org.team100.lib.logging.LoggerFactory.GlobalVelocityR3Logger;
import org.team100.lib.logging.LoggerFactory.ModelR3Logger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.motion.swerve.module.state.SwerveModulePositions;
import org.team100.lib.motion.swerve.module.state.SwerveModuleStates;
import org.team100.lib.music.Music;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.SubsystemR3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase implements SubsystemR3, Music {
    // DEBUG produces a LOT of output, you should only enable it while you're
    // looking at it.
    private static final boolean DEBUG = false;
    private final FreshSwerveEstimate m_estimate;
    private final OdometryUpdater m_odometryUpdater;
    private final SwerveLocal m_swerveLocal;
    private final SwerveLimiter m_limiter;

    // CACHES
    private final CotemporalCache<ModelR3> m_stateCache;

    // LOGGERS
    private final ModelR3Logger m_log_state;
    private final DoubleLogger m_log_turning;
    private final DoubleArrayLogger m_log_pose_array;
    private final DoubleArrayLogger m_log_field_robot;
    private final EnumLogger m_log_skill;
    private final GlobalVelocityR3Logger m_log_input;

    public SwerveDriveSubsystem(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            OdometryUpdater odometryUpdater,
            FreshSwerveEstimate estimate,
            SwerveLocal swerveLocal,
            SwerveLimiter limiter) {
        LoggerFactory child = parent.type(this);
        m_estimate = estimate;
        m_odometryUpdater = odometryUpdater;
        m_swerveLocal = swerveLocal;
        m_limiter = limiter;
        m_stateCache = Cache.of(this::update);
        stop();
        m_log_state = child.modelR3Logger(Level.COMP, "state");
        m_log_turning = child.doubleLogger(Level.TRACE, "Tur Deg");
        m_log_pose_array = child.doubleArrayLogger(Level.COMP, "pose array");
        m_log_field_robot = fieldLogger.doubleArrayLogger(Level.COMP, "robot");
        m_log_skill = child.enumLogger(Level.TRACE, "skill level");
        m_log_input = child.globalVelocityR3Logger(Level.TRACE, "drive input");
    }

    ////////////////
    //
    // ACTUATORS
    //

    /** Skip all scaling, limits generator, etc. */
    public void setVelocity(GlobalVelocityR3 input) {
        // keep the limiter up to date on what we're doing
        m_limiter.updateSetpoint(input);

        Rotation2d theta = getPose().getRotation();
        ChassisSpeeds targetChassisSpeeds = SwerveKinodynamics.toInstantaneousChassisSpeeds(
                input, theta);
        m_swerveLocal.setChassisSpeeds(targetChassisSpeeds);
        m_log_input.log(() -> input);
    }

    /**
     * Scales the supplied ChassisSpeed by the driver speed modifier.
     * 
     * @param speeds in robot coordinates
     */
    public void setChassisSpeeds(final ChassisSpeeds speeds) {
        // scale for driver skill; default is half speed.
        DriverSkill.Level driverSkillLevel = DriverSkill.level();
        m_swerveLocal.setChassisSpeeds(speeds.times(driverSkillLevel.scale()));
    }

    /**
     * Does not desaturate or optimize.
     * 
     * This "raw" mode is just for testing.
     */
    public void setRawModuleStates(SwerveModuleStates states) {
        m_swerveLocal.setRawModuleStates(states);
    }

    /** Use raw mode to set modules driving ahead */
    public Command aheadSlow() {
        return run(() -> setRawModuleStates(SwerveModuleStates.aheadSlow))
                .finallyDo(() -> stop());
    }

    /** Use robot-relative mode to set modules driving to the right */
    public Command rightwardSlow() {
        return run(() -> setChassisSpeeds(new ChassisSpeeds(0, -1.0, 0)))
                .finallyDo(() -> stop());
    }

    /** Make an X, stopped. */
    public void defense() {
        m_swerveLocal.defense();
    }

    /** Wheels ahead, stopped, for testing. */
    public void steer0() {
        m_swerveLocal.steer0();
    }

    /** Wheels at 90 degrees, stopped, for testing. */
    public void steer90() {
        m_swerveLocal.steer90();
    }

    @Override
    public void stop() {
        m_swerveLocal.stop();
    }

    public void resetPose(Pose2d robotPose) {
        if (DEBUG)
            System.out.println("WARNING: Make sure resetting the swerve module collection doesn't break anything");
        m_swerveLocal.reset();
        m_odometryUpdater.reset(robotPose);
        m_stateCache.reset();
    }

    ///////////////////////////////////////////////////////////////
    //
    // Observers
    //

    /**
     * Cached.
     * 
     * SwerveState representing the drivetrain's field-relative pose, velocity, and
     * acceleration.
     */
    @Override
    public ModelR3 getState() {
        return m_stateCache.get();
    }

    ///////////////////////////////////////////////////////////////

    /**
     * Periodic() should not do actuation. Let commands do that.
     */
    @Override
    public void periodic() {
        if (DEBUG)
            System.out.println("drive periodic");
        // m_poseEstimator.periodic();
        // 4/2/25 Joel removed this state resetter because it happens earlier in
        // Robot.java
        // and i think we don't need to do it twice.
        // m_stateSupplier.reset();
        m_log_state.log(this::getState);
        m_log_turning.log(() -> getPose().getRotation().getDegrees());
        m_log_pose_array.log(this::poseArray);

        // Update the Field2d widget
        // the name "field" is used by Field2d.
        // the name "robot" can be anything.
        m_log_field_robot.log(this::poseArray);
        m_log_skill.log(() -> DriverSkill.level());
        m_swerveLocal.periodic();
    }

    private double[] poseArray() {
        return new double[] {
                getPose().getX(),
                getPose().getY(),
                getPose().getRotation().getDegrees()
        };
    }

    public void close() {
        m_swerveLocal.close();
    }

    /////////////////////////////////////////////////////////////////

    /**
     * Compute the current state. This is a fairly heavyweight thing to do, so it
     * should be cached (thus refreshed once per cycle).
     */
    private ModelR3 update() {
        double now = Takt.get();
        SwerveModulePositions positions = m_swerveLocal.positions();
        // now that the pose estimator uses the SideEffect thing, we don't need this.
        // m_odometryUpdater.update();
        // m_cameraUpdater.run();
        ModelR3 swerveModel = m_estimate.apply(now);
        if (DEBUG) {
            System.out.printf("update() positions %s estimated pose: %s\n", positions, swerveModel);
        }
        return swerveModel;
    }

    /** Return cached pose. */
    public Pose2d getPose() {
        return m_stateCache.get().pose();
    }

    /** Return cached velocity. */
    public GlobalVelocityR3 getVelocity() {
        return m_stateCache.get().velocity();
    }

    /** Return cached speeds. */
    public ChassisSpeeds getChassisSpeeds() {
        return m_stateCache.get().chassisSpeeds();
    }

    @Override
    public Command play(double freq) {
        return run(() -> {
            m_swerveLocal.play(freq);
        });
    }

}

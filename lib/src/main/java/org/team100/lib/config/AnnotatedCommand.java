package org.team100.lib.config;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command with annotations that are checked against ground truth while
 * disabled.
 * 
 * If you don't care about the annotations at all, e.g. because the same command
 * works on both sides from anywhere, use null.
 */
public record AnnotatedCommand(Command command, Alliance alliance, Pose2d start) {
}

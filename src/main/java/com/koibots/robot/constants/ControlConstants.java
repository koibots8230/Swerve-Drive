package com.koibots.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class ControlConstants {

  public static final double DEADBAND = 0.02;

  private static final double ROBOT_WIDTH_METERS = Units.inchesToMeters(30);
  private static final double ROBOT_LENGTH_METERS = Units.inchesToMeters(30);

  public static final SwerveDriveKinematics SWERVE_KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(ROBOT_LENGTH_METERS / 2, ROBOT_WIDTH_METERS / 2), // Front Left
          new Translation2d(ROBOT_LENGTH_METERS / 2, -ROBOT_WIDTH_METERS / 2), // Front Right
          new Translation2d(-ROBOT_LENGTH_METERS / 2, ROBOT_WIDTH_METERS / 2), // Back Left
          new Translation2d(-ROBOT_LENGTH_METERS / 2, -ROBOT_WIDTH_METERS / 2) // Front Right
          );
}

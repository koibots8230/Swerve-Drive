package com.koibots.robot;

import static java.lang.StrictMath.PI;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Constants {

  public static final int MAX_LINEAR_SPEED_METERS_PER_SECOND = 4; // Meters per Second
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 2 * PI; // Radians per Second

  public static final double DEADBAND = 0.02;

  private static final double ROBOT_WIDTH_METERS = Units.inchesToMeters(30);
  private static final double ROBOT_LENGTH_METERS = Units.inchesToMeters(30);

  public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(ROBOT_LENGTH_METERS / 2, ROBOT_WIDTH_METERS / 2), // Front Left
      new Translation2d(ROBOT_LENGTH_METERS / 2, -ROBOT_WIDTH_METERS / 2), // Front Right
      new Translation2d(-ROBOT_LENGTH_METERS / 2, ROBOT_WIDTH_METERS / 2), // Back Left
      new Translation2d(-ROBOT_LENGTH_METERS / 2, -ROBOT_WIDTH_METERS / 2) // Front Right
  );

  public static final boolean SET_REPLAY = false;
  public static final Field2d FIELD = new Field2d();
}

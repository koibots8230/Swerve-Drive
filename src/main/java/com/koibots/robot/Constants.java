package com.koibots.robot;

import static java.lang.StrictMath.PI;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import static edu.wpi.first.units.Units.*;

public class Constants {
    public static double PERIOD = 0.02;
    public static final double DEADBAND = 0.02;
    public static final boolean SET_REPLAY = false;

    public static class ControllerConstants {

    }

    public static final class DrivetrainConstants {
        // CAN Ids
        public static int FRONT_LEFT_DRIVE_ID = 0;
        public static int FRONT_LEFT_TURN_ID = 0;
        public static int FRONT_RIGHT_DRIVE_ID = 0;
        public static int FRONT_RIGHT_TURN_ID = 0;
        public static int BACK_LEFT_DRIVE_ID = 0;
        public static int BACK_LEFT_TURN_ID = 0;
        public static int BACK_RIGHT_DRIVE_ID = 0;
        public static int BACK_RIGHT_TURN_ID = 0;

        // Dimensions
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
        private static final Measure<Distance> ROBOT_WIDTH_METERS = Inches.of(39);
        private static final Measure<Distance> ROBOT_LENGTH_METERS = Inches.of(30);


        // Constraints
        public static final double MAX_MODULE_LINEAR_SPEED = 12;
        public static final double MAX_TRANSLATION_SPEED = 12;
        public static final double MAX_ANGULAR_VELOCITY = 2 * Math.PI;

        /**
         * Modules are ordered FL-FR-BL-BR
         *  0 - Front Left
         *  1 - Front Right
         *  2 - Back Left
         *  3 - Back Right
         */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(ROBOT_LENGTH_METERS.in(Meters) / 2, ROBOT_WIDTH_METERS.in(Meters) / 2),
                new Translation2d(ROBOT_LENGTH_METERS.in(Meters) / 2, -ROBOT_WIDTH_METERS.in(Meters) / 2),
                new Translation2d(-ROBOT_LENGTH_METERS.in(Meters) / 2, ROBOT_WIDTH_METERS.in(Meters) / 2),
                new Translation2d(-ROBOT_LENGTH_METERS.in(Meters) / 2, -ROBOT_WIDTH_METERS.in(Meters) / 2)
        );

    }
}

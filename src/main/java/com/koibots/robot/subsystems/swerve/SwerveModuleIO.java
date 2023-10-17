package com.koibots.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleIOState {
        double velocityMetersPerSecond;
        double angle;
        double distance;
        SwerveModuleState moduleState;

        public SwerveModulePosition getModulePosition() {
            return new SwerveModulePosition(distance, Rotation2d.fromDegrees(angle));
        }

        public SwerveModuleState getModuleState() {
            return new SwerveModuleState(velocityMetersPerSecond, Rotation2d.fromDegrees(angle));
        }
    }

    void updateState(SwerveModuleIOState state);

    void setDesiredState(SwerveModuleState state);

    void resetEncoders();

}
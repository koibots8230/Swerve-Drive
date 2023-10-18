package com.koibots.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleIOState {
        double angle;
        double distance;



    }

    void updateState(SwerveModuleIOState state);

    void setDesiredState(SwerveModuleState state);

    void resetEncoders();

    SwerveModulePosition getModulePosition();

}
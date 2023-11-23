package com.koibots.robot.subsystems.swerve;

import com.koibots.robot.Constants.DrivetrainConstants;
import com.koibots.robot.Robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
    SwerveModule[] swerveModules;
    GyroIO gyro;
    GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    SwerveDrivePoseEstimator odometry;

    public Swerve() {
        switch (Robot.getMode()) {
            case REAL:
                swerveModules = new SwerveModule[] {
                        new SwerveModule(new SwerveModuleIOSparkMax(
                                DrivetrainConstants.FRONT_LEFT_DRIVE_ID,
                                DrivetrainConstants.FRONT_LEFT_TURN_ID), "FrontLeft"),
                        new SwerveModule(new SwerveModuleIOSparkMax(
                                DrivetrainConstants.FRONT_RIGHT_DRIVE_ID,
                                DrivetrainConstants.FRONT_RIGHT_TURN_ID), "FrontRight"),
                        new SwerveModule(new SwerveModuleIOSparkMax(
                                DrivetrainConstants.BACK_LEFT_DRIVE_ID,
                                DrivetrainConstants.BACK_LEFT_TURN_ID), "BackLeft"),
                        new SwerveModule(new SwerveModuleIOSparkMax(
                                DrivetrainConstants.BACK_RIGHT_DRIVE_ID,
                                DrivetrainConstants.BACK_RIGHT_TURN_ID), "BackRight"),
                };

                gyro = new GyroIONavX();
                break;
            case SIM:
                swerveModules = new SwerveModule[] {
                        new SwerveModule(new SwerveModuleIOSim(), "FrontLeft"),
                        new SwerveModule(new SwerveModuleIOSim(), "FrontRight"),
                        new SwerveModule(new SwerveModuleIOSim(), "BackLeft"),
                        new SwerveModule(new SwerveModuleIOSim(), "BackRight"),
                };

                gyro = new GyroIOSim();
                break;
            case REPLAY:
                break;
        }

        gyro.zeroYaw();

        odometry = new SwerveDrivePoseEstimator(
                DrivetrainConstants.SWERVE_KINEMATICS,
                new Rotation2d(),
                getModulePositions(),
                new Pose2d()
        );
    }

    @Override
    public void periodic() {
        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        odometry.update(gyroInputs.yawPosition, getModulePositions());

        Logger.recordOutput("Odometry", odometry.getEstimatedPosition());

        swerveModules[0].periodic();
        swerveModules[1].periodic();
        swerveModules[2].periodic();
        swerveModules[3].periodic();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            swerveModules[0].stop();
            swerveModules[1].stop();
            swerveModules[2].stop();
            swerveModules[3].stop();
        }

        // Log measured states
        Logger.recordOutput("SwerveStates/Measured", getModuleStates());
    }

    public void setModuleStates(SwerveModuleState[] states) {
        Logger.recordOutput("Module Setpoints", states);

        swerveModules[0].setState(states[0]);
        swerveModules[1].setState(states[1]);
        swerveModules[2].setState(states[2]);
        swerveModules[3].setState(states[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                swerveModules[0].getState(),
                swerveModules[1].getState(),
                swerveModules[2].getState(),
                swerveModules[3].getState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition()
        };
    }

    public Pose2d getEstimatedPose() {
        return odometry.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(gyroInputs.yawPosition, getModulePositions(), pose);
    }

    public void stop() {
        setModuleStates( new SwerveModuleState[]{
                new SwerveModuleState(0, getModuleStates()[0].angle),
                new SwerveModuleState(0, getModuleStates()[1].angle),
                new SwerveModuleState(0, getModuleStates()[2].angle),
                new SwerveModuleState(0, getModuleStates()[3].angle),
        });
    }
}

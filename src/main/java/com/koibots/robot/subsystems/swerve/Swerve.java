package com.koibots.robot.subsystems.swerve;


import com.koibots.robot.Robot;
import com.koibots.robot.constants.ControlConstants;
import com.koibots.robot.constants.SimConstants;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

                break;
            case SIM:
                swerveModules = new SwerveModule[] {
                        new SwerveModule(new SwerveModuleIOSim(), 0),
                        new SwerveModule(new SwerveModuleIOSim(), 1),
                        new SwerveModule(new SwerveModuleIOSim(), 2),
                        new SwerveModule(new SwerveModuleIOSim(), 3)
                };

                gyro = new GyroIO() {};

                odometry = new SwerveDrivePoseEstimator(
                        ControlConstants.SWERVE_KINEMATICS,
                        new Rotation2d(),
                        getModulePositions(),
                        new Pose2d()
                );

                break;
            case REPLAY:

        }
    }

    @Override
    public void periodic() {
        simulationPeriodic();

        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
    
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

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
        Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
        Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        odometry.update(gyroInputs.yawPosition, getModulePositions());
        Logger.recordOutput("Odometry/Robot", getEstimatedPose());
    }

    public void simulationPeriodic() {
        ChassisSpeeds simSpeeds = ControlConstants.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());

        Logger.recordOutput("Calculated Speeds", new double[] {simSpeeds.vxMetersPerSecond, simSpeeds.vyMetersPerSecond, simSpeeds.omegaRadiansPerSecond});

        gyroInputs.yawPosition = gyroInputs.yawPosition.plus(Rotation2d.fromRadians(simSpeeds.omegaRadiansPerSecond * 0.02));
        gyroInputs.yawVelocityRadPerSec = simSpeeds.omegaRadiansPerSecond;

        SimConstants.FIELD.setRobotPose(getEstimatedPose());
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
}

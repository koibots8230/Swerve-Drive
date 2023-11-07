package com.koibots.robot.subsystems.swerve;


import com.koibots.robot.Robot;
import com.koibots.robot.constants.ControlConstants;
import com.koibots.robot.constants.PhysicalConstants;
import com.koibots.robot.constants.SimConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import static edu.wpi.first.math.kinematics.SwerveDriveKinematics.desaturateWheelSpeeds;


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

    public void setModuleStates(SwerveModuleState[] states) {
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

    @Override
    public void simulationPeriodic() {
        ChassisSpeeds simSpeeds = ControlConstants.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());

        gyroInputs.yawPosition.plus(Rotation2d.fromRadians(simSpeeds.omegaRadiansPerSecond));
        gyroInputs.yawVelocityRadPerSec = simSpeeds.omegaRadiansPerSecond;

        SimConstants.FIELD.setRobotPose(getEstimatedPose());
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

    public class TeleopCommand extends Command {


        DoubleSupplier vxSupplier;
        DoubleSupplier vySupplier;
        DoubleSupplier vThetaSupplier;
        DoubleSupplier angleSupplier;
        BooleanSupplier crossSupplier;
        Function<Double, Double> scalingFunction;
        
        double previousTimestamp;

        public TeleopCommand(
                DoubleSupplier vxSupplier,
                DoubleSupplier vySupplier,
                DoubleSupplier vThetaSupplier,
                DoubleSupplier angleSupplier,
                BooleanSupplier crossSupplier
        ) {
            this.vxSupplier = vxSupplier;
            this.vySupplier = vySupplier;
            this.vThetaSupplier = vThetaSupplier;
            this.angleSupplier = angleSupplier;
            this.crossSupplier = crossSupplier;

            

            addRequirements(Swerve.this);
        }

        public void setScalingAlgorithm(Function<Double, Double> algorithm) {
            this.scalingFunction = algorithm;
        }

        @Override
        public void initialize() {
            previousTimestamp = Logger.getRealTimestamp();

            System.out.println("Swerve Teleop Command Initialized");
        }

        @Override
        public void execute() {
            if (!this.crossSupplier.getAsBoolean()) { // Normal Field Oriented
                double linearMagnitude =
                        MathUtil.applyDeadband(
                                Math.hypot(vxSupplier.getAsDouble(), vySupplier.getAsDouble()),
                                ControlConstants.DEADBAND,
                                1
                        );

                Rotation2d linearDirection =
                        new Rotation2d(vxSupplier.getAsDouble(), vySupplier.getAsDouble());

                double angularVelocity = MathUtil.applyDeadband(vThetaSupplier.getAsDouble(), ControlConstants.DEADBAND);
                // TODO: Implement POV for angle alignment

                // Apply Scaling
                linearMagnitude = Math.abs(scalingFunction.apply(linearMagnitude));
                angularVelocity = scalingFunction.apply(angularVelocity);

                Translation2d linearVelocity =
                        new Pose2d(new Translation2d(), linearDirection)
                                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                                .getTranslation();

                ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.getX() * PhysicalConstants.MAX_LINEAR_SPEED,
                        linearVelocity.getY() * PhysicalConstants.MAX_LINEAR_SPEED,
                        angularVelocity * PhysicalConstants.MAX_ANGULAR_VELOCITY,
                        Swerve.this.getEstimatedPose().getRotation()
                );

                double periodSeconds = Logger.getRealTimestamp() - previousTimestamp;

                Twist2d setpointTwist = new Pose2d()
                        .log(
                                new Pose2d(
                                        speeds.vxMetersPerSecond * periodSeconds,
                                        speeds.vyMetersPerSecond * periodSeconds,
                                        new Rotation2d(speeds.omegaRadiansPerSecond * periodSeconds)));

                ChassisSpeeds adjustedSpeeds =
                        new edu.wpi.first.math.kinematics.ChassisSpeeds(
                                setpointTwist.dx / periodSeconds,
                                setpointTwist.dy / periodSeconds,
                                setpointTwist.dtheta / periodSeconds);

                SwerveModuleState[] targetModuleStates = ControlConstants.SWERVE_KINEMATICS.toSwerveModuleStates(adjustedSpeeds);

                desaturateWheelSpeeds(targetModuleStates, PhysicalConstants.MAX_LINEAR_SPEED);

                if (adjustedSpeeds.vxMetersPerSecond == 0.0
                        && adjustedSpeeds.vyMetersPerSecond == 0.0
                        && adjustedSpeeds.omegaRadiansPerSecond == 0) {
                    targetModuleStates[0] = new SwerveModuleState(0, swerveModules[0].getAngle());
                    targetModuleStates[1] = new SwerveModuleState(0, swerveModules[1].getAngle());
                    targetModuleStates[2] = new SwerveModuleState(0, swerveModules[2].getAngle());
                    targetModuleStates[3] = new SwerveModuleState(0, swerveModules[3].getAngle());
                }

                Swerve.this.setModuleStates(targetModuleStates);
            } else { // Set Cross

            }
        }
    }
}

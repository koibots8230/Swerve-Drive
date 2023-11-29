package com.koibots.robot.command;

import com.koibots.robot.Constants;
import com.koibots.robot.Constants.DrivetrainConstants;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

public class FieldOrientedDrive extends Command {
    DoubleSupplier vxSupplier;
    DoubleSupplier vySupplier;
    DoubleSupplier vThetaSupplier;
    DoubleSupplier angleSupplier;
    BooleanSupplier crossSupplier;
    Function<Double, Double> scalingAlgorithm;

    ProfiledPIDController angleAlignmentController;

    public FieldOrientedDrive(
            DoubleSupplier vxSupplier,
            DoubleSupplier vySupplier,
            DoubleSupplier vThetaSupplier,
            DoubleSupplier angleSupplier,
            BooleanSupplier crossSupplier,
            Function<Double, Double> scalingAlgorithm) {
        this.vxSupplier = vxSupplier;
        this.vySupplier = vySupplier;
        this.vThetaSupplier = vThetaSupplier;
        this.angleSupplier = angleSupplier;
        this.crossSupplier = crossSupplier;
        this.scalingAlgorithm = scalingAlgorithm;

        angleAlignmentController = new ProfiledPIDController(
                0.2,
                0,
                0,
                new Constraints(DrivetrainConstants.MAX_ANGULAR_VELOCITY, 4 * Math.PI),
                0.02);

        angleAlignmentController.enableContinuousInput(0, 2 * Math.PI);

        addRequirements(Swerve.get());
    }

    @Override
    public void execute() {
        Logger.recordOutput(
                "Swerve Command Inputs",
                new double[] {
                        vxSupplier.getAsDouble(), vySupplier.getAsDouble(),
                        vThetaSupplier.getAsDouble()
                });

        if (!this.crossSupplier.getAsBoolean()) { // Normal Field Oriented Drive
            double linearMagnitude = MathUtil.applyDeadband(
                    Math.hypot(vxSupplier.getAsDouble(), vySupplier.getAsDouble()),
                    Constants.DEADBAND,
                    1);

            Rotation2d linearDirection = new Rotation2d(vxSupplier.getAsDouble(), vySupplier.getAsDouble());

            double angularVelocity;

            if (angleSupplier.getAsDouble() != -1) {
                angularVelocity = angleAlignmentController.calculate(
                        Swerve.get().getEstimatedPose().getRotation().getRadians(),
                        -Math.toRadians(angleSupplier.getAsDouble()));
            } else {
                angularVelocity = MathUtil.applyDeadband(vThetaSupplier.getAsDouble(),
                        Constants.DEADBAND);
            }

            // Apply Scaling
            linearMagnitude = scalingAlgorithm.apply(linearMagnitude);
            // angularVelocity = scalingFunction.apply(angularVelocity);

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearMagnitude * linearDirection.getCos()
                            * DrivetrainConstants.MAX_TRANSLATION_SPEED,//.in(MetersPerSecond),
                    linearMagnitude * linearDirection.getSin()
                            * DrivetrainConstants.MAX_TRANSLATION_SPEED,//.in(MetersPerSecond),
                    angularVelocity * DrivetrainConstants.MAX_ANGULAR_VELOCITY,//.in(RadiansPerSecond),
                    Swerve.get().getEstimatedPose().getRotation());

            speeds = ChassisSpeeds.discretize(speeds, Constants.PERIOD);

            SwerveModuleState[] targetModuleStates = DrivetrainConstants.SWERVE_KINEMATICS
                    .toSwerveModuleStates(speeds);

            SwerveDriveKinematics.desaturateWheelSpeeds(
                    targetModuleStates,
                    speeds,
                    Constants.DrivetrainConstants.MAX_MODULE_LINEAR_SPEED,
                    Constants.DrivetrainConstants.MAX_TRANSLATION_SPEED,
                    Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY
            );

            if (speeds.vxMetersPerSecond == 0.0
                    && speeds.vyMetersPerSecond == 0.0
                    && speeds.omegaRadiansPerSecond == 0) {
                var currentStates = Swerve.get().getModuleStates();
                targetModuleStates[0] = new SwerveModuleState(0, currentStates[0].angle);
                targetModuleStates[1] = new SwerveModuleState(0, currentStates[1].angle);
                targetModuleStates[2] = new SwerveModuleState(0, currentStates[2].angle);
                targetModuleStates[3] = new SwerveModuleState(0, currentStates[3].angle);
            }

            Swerve.get().setModuleStates(targetModuleStates);
        } else { // Set Cross
            Swerve.get().setModuleStates(
                    new SwerveModuleState[] {
                            new SwerveModuleState(0,
                                    Rotation2d.fromDegrees(45)),
                            new SwerveModuleState(0,
                                    Rotation2d.fromDegrees(-45)),
                            new SwerveModuleState(0,
                                    Rotation2d.fromDegrees(-45)),
                            new SwerveModuleState(0,
                                    Rotation2d.fromDegrees(45))
                    });
        }
    }
}

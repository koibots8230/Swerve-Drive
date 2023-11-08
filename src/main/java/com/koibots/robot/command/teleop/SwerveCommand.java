package com.koibots.robot.command.teleop;

import com.koibots.robot.constants.ControlConstants;
import com.koibots.robot.constants.PhysicalConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.math.kinematics.SwerveDriveKinematics.desaturateWheelSpeeds;

public class SwerveCommand extends Command {
    DoubleSupplier vxSupplier;
    DoubleSupplier vySupplier;
    DoubleSupplier vThetaSupplier;
    DoubleSupplier angleSupplier;
    BooleanSupplier crossSupplier;
    Function<Double, Double> scalingFunction;

    double previousTimestamp;

    public SwerveCommand(
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

        addRequirements(Swerve.get());
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
        Logger.recordOutput("Swerve Command Inputs", new double[] {
                vxSupplier.getAsDouble(), vySupplier.getAsDouble(), vThetaSupplier.getAsDouble()
        });


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
            linearMagnitude = scalingFunction.apply(linearMagnitude);
            //angularVelocity = scalingFunction.apply(angularVelocity);

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearMagnitude * linearDirection.getCos() * PhysicalConstants.MAX_LINEAR_SPEED,
                    linearMagnitude * linearDirection.getSin() * PhysicalConstants.MAX_LINEAR_SPEED,
                    angularVelocity * PhysicalConstants.MAX_ANGULAR_VELOCITY,
                    Swerve.get().getEstimatedPose().getRotation()
            );

            double periodSeconds = Logger.getRealTimestamp() - previousTimestamp;

            ChassisSpeeds.discretize(speeds, periodSeconds);

            SwerveModuleState[] targetModuleStates = ControlConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

            desaturateWheelSpeeds(targetModuleStates, PhysicalConstants.MAX_LINEAR_SPEED);

            if (speeds.vxMetersPerSecond == 0.0
                    && speeds.vyMetersPerSecond == 0.0
                    && speeds.omegaRadiansPerSecond == 0) {
                var currentStates = Swerve.get().getModuleStates();
                targetModuleStates[0] = new SwerveModuleState(0, currentStates[0].angle);
                targetModuleStates[1] = new SwerveModuleState(0, currentStates[1].angle);
                targetModuleStates[2] = new SwerveModuleState(0, currentStates[2].angle);
                targetModuleStates[3] = new SwerveModuleState(0, currentStates[3].angle);
            }
            
            previousTimestamp = Logger.getRealTimestamp();
            Swerve.get().setModuleStates(targetModuleStates);
        } else { // Set Cross

        }
    }
}

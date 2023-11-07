package com.koibots.robot.subsystems.swerve;


import com.koibots.robot.Robot;
import com.koibots.robot.constants.ControlConstants;
import com.koibots.robot.constants.PhysicalConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import static edu.wpi.first.math.kinematics.SwerveDriveKinematics.desaturateWheelSpeeds;


public class Swerve extends SubsystemBase {
    SwerveModule[] swerveModules;
    GyroIO gyro;
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

                odometry = new SwerveDrivePoseEstimator(
                    ControlConstants.SWERVE_KINEMATICS,
                    new Rotation2d(),

            );

                break;
            case REPLAY:

        }
    }

    public void setModuleStates(SwerveModuleState[] states) {
        swerveModules[0].setState(states[0]);
        swerveModules[1].setState(states[1]);
        swerveModules[2].setState(states[2]);
        swerveModules[3].setState(states[3]);
    }

    public class TeleopCommand extends CommandBase {

        // Graph of algorithms here: https://www.desmos.com/calculator/w738aldioj
        // TODO: Clean these up
        enum ScalingAlgorithm {
            Linear(
                    (x) -> x
            ),
            Squared(
                    (x) -> Math.signum(x) * x * x
            ),

            Cubed(
                    (x) -> x * x * x
            ),

            Cosine(
                    (x) -> (-Math.signum(x) * Math.cos(Math.PI * 0.5 * x )) + (1 * Math.signum(x))
            ),

            CubedSquareRoot(
                    (x) -> Math.signum(x) * Math.sqrt(Math.abs(x * x * x))
            );

            public final Function<Double, Double> algorithm;

            private ScalingAlgorithm(Function<Double, Double> algorithm) {
                this.algorithm = algorithm;
            }
        }


        DoubleSupplier vxSupplier;
        DoubleSupplier vySupplier;
        DoubleSupplier vThetaSupplier;
        DoubleSupplier angleSupplier;
        BooleanSupplier crossSupplier;
        Function<Double, Double> scalingFunction;
        LoggedDashboardChooser<ScalingAlgorithm> scalingChooser;
        double previousTimestamp;

        TeleopCommand(
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

            scalingChooser.addDefaultOption("Linear", ScalingAlgorithm.Linear);
            scalingChooser.addOption("Squared", ScalingAlgorithm.Squared);
            scalingChooser.addOption("Cubed", ScalingAlgorithm.Cubed);
            scalingChooser.addOption("Cosine", ScalingAlgorithm.Cosine);
            scalingChooser.addOption("Fancy", ScalingAlgorithm.CubedSquareRoot);

            addRequirements(Swerve.this);

            this.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        }

        @Override
        public void initialize() {
            scalingFunction = scalingChooser.get().algorithm;

            previousTimestamp = Logger.getInstance().getRealTimestamp();
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
                        new Rotation2d() // TODO: Put actual estimated angle
                );

                double periodSeconds = Logger.getInstance().getRealTimestamp() - previousTimestamp;

                var setpointTwist = new Pose2d()
                        .log(
                                new Pose2d(
                                        speeds.vxMetersPerSecond * periodSeconds,
                                        speeds.vyMetersPerSecond * periodSeconds,
                                        new Rotation2d(speeds.omegaRadiansPerSecond * periodSeconds)));

                var adjustedSpeeds =
                        new edu.wpi.first.math.kinematics.ChassisSpeeds(
                                setpointTwist.dx / periodSeconds,
                                setpointTwist.dy / periodSeconds,
                                setpointTwist.dtheta / periodSeconds);

                var targetModuleStates = ControlConstants.SWERVE_KINEMATICS.toSwerveModuleStates(adjustedSpeeds);

                desaturateWheelSpeeds(targetModuleStates, PhysicalConstants.MAX_LINEAR_SPEED);

                if (adjustedSpeeds.vxMetersPerSecond == 0.0
                        && adjustedSpeeds.vyMetersPerSecond == 0.0
                        && adjustedSpeeds.omegaRadiansPerSecond == 0) {
                    // TODO: Replace Rotation2d with actual previous state
                    targetModuleStates[0] = new SwerveModuleState(0, new Rotation2d());
                    targetModuleStates[1] = new SwerveModuleState(0, new Rotation2d());
                    targetModuleStates[2] = new SwerveModuleState(0, new Rotation2d());
                    targetModuleStates[3] = new SwerveModuleState(0, new Rotation2d());
                }

                Swerve.this.setModuleStates(targetModuleStates);
            } else { // Set Cross

            }
        }
    }
}

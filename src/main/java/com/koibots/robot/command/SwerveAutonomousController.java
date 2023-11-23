package com.koibots.robot.command;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

import com.koibots.lib.trajectory.Trajectory;
import com.koibots.lib.trajectory.TrajectoryState;
import com.koibots.robot.Constants;
import com.koibots.robot.Constants.DrivetrainConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class SwerveAutonomousController extends Command {
    double initTime;
    boolean stop;
    Trajectory trajectory;
    PIDController xErrorController;
    PIDController yErrorController;
    PIDController headingErrorController;

    SwerveAutonomousController(Trajectory trajectory, boolean stop) {
        this.trajectory = trajectory;
        this.stop = stop;
        addRequirements(Swerve.get());
    }

    @Override
    public void initialize() {
        initTime = Logger.getRealTimestamp();

        Swerve.get().resetOdometry(trajectory.getInitialPose());

        xErrorController = new PIDController(0, 0, 0, Constants.PERIOD);
        yErrorController = new PIDController(0, 0, 0, Constants.PERIOD);
        headingErrorController = new PIDController(0, 0, 0, Constants.PERIOD);
    }

    @Override
    public void execute() {
        TrajectoryState setpoints = trajectory.sample(Logger.getRealTimestamp() - initTime);
        Pose2d measuredPose = Swerve.get().getEstimatedPose();

        ChassisSpeeds speeds = new ChassisSpeeds(
                setpoints.velocityX
                        + xErrorController.calculate(measuredPose.getX(), setpoints.x),
                setpoints.velocityY
                        + yErrorController.calculate(measuredPose.getY(), setpoints.y),
                setpoints.angularVelocity
                        + headingErrorController.calculate(
                                measuredPose.getRotation().getRadians(),
                                setpoints.heading
                )
        );

        speeds = ChassisSpeeds.discretize(speeds, Constants.PERIOD);

        SwerveModuleState[] targetStates = DrivetrainConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
                targetStates,
                speeds,
                Constants.DrivetrainConstants.MAX_MODULE_LINEAR_SPEED,
                Constants.DrivetrainConstants.MAX_TRANSLATION_SPEED,
                Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY
        );

        Swerve.get().setModuleStates(targetStates);
    }

    @Override
    public boolean isFinished() {
        return trajectory.completed();
    }

    @Override
    public void end(boolean interrupted) {
        if (stop) {
            Swerve.get().stop();
        }
    }
}

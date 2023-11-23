package com.koibots.robot.command;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

import com.koibots.lib.trajectory.Trajectory;
import com.koibots.lib.trajectory.TrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import org.littletonrobotics.junction.Logger;

public class SwerveAutonomousController extends Command {
    double initTime;
    boolean stop;
    Trajectory trajectory;

    SwerveAutonomousController(Trajectory trajectory, boolean stop) {
        this.trajectory = trajectory;
        this.stop = stop;
        addRequirements(Swerve.get());
    }

    @Override
    public void initialize() {
        initTime = Logger.getRealTimestamp();
        Swerve.get().resetOdometry(trajectory.getInitialPose());
    }

    @Override
    public void execute() {
        TrajectoryState setpoints = trajectory.sample(Logger.getRealTimestamp() - initTime);

        Twist2d error = Swerve.get().getEstimatedPose().log(
                new Pose2d(setpoints.x, setpoints.y, Rotation2d.fromRadians(setpoints.heading)));

        // TODO Implement auto control
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

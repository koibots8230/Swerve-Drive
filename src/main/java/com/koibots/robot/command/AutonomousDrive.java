package com.koibots.robot.command;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.io.IOException;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

public class AutonomousDrive extends CommandBase {
    Trajectory auto;
    double initialTimeSeconds;
    double autoRunTime;

    public AutonomousDrive() {
        try {
            auto = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath());
            autoRunTime = auto.getTotalTimeSeconds();
        } catch (IOException e) {
            DriverStation.reportError("Failed to find auto file", false);
        }
    }

    @Override
    public void initialize() {
        initialTimeSeconds = Timer.getFPGATimestamp();
        Swerve.get().trajectoryFollower.setEnabled(true);
        Swerve.get().resetOdometry(auto.getInitialPose());
    }

    @Override
    public void execute() {
        var target = auto.sample(Timer.getFPGATimestamp() - initialTimeSeconds);


        Swerve.get().trajectoryFollower.calculate(
                Swerve.get().getEstimatedPosition(),
                target,
                new Rotation2d()
        );
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.get().setCross();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - initialTimeSeconds >= autoRunTime &&
                Swerve.get().trajectoryFollower.atReference();
    }
}

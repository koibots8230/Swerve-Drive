package com.koibots.robot.command;

import com.koibots.robot.subsystems.swerve.Swerve;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static com.koibots.lib.math.SwerveUtils.secondOrderKinematics;
import static com.koibots.robot.constants.Constants.DriveConstants.*;
import static com.koibots.robot.constants.Constants.DriveConstants.MAX_SPEED_METERS_PER_SECOND;
import static com.koibots.robot.constants.Constants.PERIOD;

public class FieldOrientedDrive extends CommandBase {
    private final DoubleSupplier vxSupplier;
    private final DoubleSupplier vySupplier;
    private final DoubleSupplier angularVelocitySupplier;
    private SwerveModuleState[] previousStates = new SwerveModuleState[4];

    public FieldOrientedDrive(
            DoubleSupplier vxSupplier,
            DoubleSupplier vySupplier,
            DoubleSupplier angularVelocitySupplier
    ) {
        this.vxSupplier = vxSupplier;
        this.vySupplier = vySupplier;
        this.angularVelocitySupplier = angularVelocitySupplier;

        addRequirements(Swerve.get());
    }

    @Override
    public void execute() {
        var currentSetpoints = secondOrderKinematics(
                new ChassisSpeeds(
                        vxSupplier.getAsDouble() * MAX_SPEED_METERS_PER_SECOND,
                        vySupplier.getAsDouble() * MAX_SPEED_METERS_PER_SECOND,
                        angularVelocitySupplier.getAsDouble() * MAX_ANGULAR_SPEED
                ),
                DRIVE_KINEMATICS,
                PERIOD,
                MAX_SPEED_METERS_PER_SECOND,
                previousStates
        );

        previousStates = currentSetpoints;

        Swerve.get().setModuleStates(currentSetpoints);
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.get().stop();
    }
}
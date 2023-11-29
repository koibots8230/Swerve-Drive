// Copyright 2023 Koibots - FRC 8230
// This program is lice

package com.koibots.robot.subsystems.swerve;

import com.koibots.robot.Constants.DrivetrainConstants;
import com.koibots.robot.Robot;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {


    private final SwerveModuleIO io;
    private final SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();
    private final String name;

    private SimpleMotorFeedforward driveFeedforward;
    private PIDController driveFeedback;
    private PIDController turnFeedback;

    public SwerveModule(SwerveModuleIO io, String name) {
        this.io = io;
        this.name = name;

        // Switch constants based on mode (the physics simulator is treated as a
        // separate robot with different tuning)
        switch (Robot.getMode()) {
            case REAL:

                break;
            case SIM:
                driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
                driveFeedback = new PIDController(0.1, 0.0, 0.0);
                turnFeedback = new PIDController(10.0, 0.0, 0.0);
                break;
            case REPLAY:
                break;
        }

        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/" + name, inputs);
    }

    /** Runs the module with the specified setpoint state. Returns the optimized state. */
    public void setState(SwerveModuleState state) {
        // Optimize state based on current angle
        // Controllers run in "periodic" when the setpoint is not null
        var optimizedState = SwerveModuleState.optimize(state, getAngle());

        io.setDriveVoltage(
                driveFeedback.calculate(inputs.driveVelocityRadPerSec, optimizedState.speedMetersPerSecond)
                        + driveFeedforward.calculate(optimizedState.speedMetersPerSecond)
        );

        io.setModuleAngle(optimizedState.angle.getRadians());
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.stopTurnMotor();
        io.setDriveVoltage(0.0);
    }

    /** Sets whether brake mode is enabled. */
    public void setBrakeMode() {
        io.setDriveIdleMode(CANSparkMax.IdleMode.kBrake);
        io.setTurnIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return inputs.modulePosition;
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return inputs.drivePositionRad * DrivetrainConstants.WHEEL_RADIUS;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * DrivetrainConstants.WHEEL_RADIUS;
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }
}

package com.koibots.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {
    CANSparkMax driveMotor;
    CANSparkMax turnMotor;
    RelativeEncoder driveEncoder;
    RelativeEncoder turnEncoder;

    public SwerveModuleIOSparkMax(int driveID, int turnID) {
        driveMotor = new CANSparkMax(driveID, CANSparkMaxLowLevel.MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnID, CANSparkMaxLowLevel.MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
        // TODO: Check through bore actual values
    }

    /** Updates the set of loggable inputs. */
    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        inputs.drivePositionRad = driveEncoder.getPosition();
        inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
        inputs.driveAppliedVolts = driveMotor.getBusVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();

        inputs.turnAbsolutePosition = Rotation2d.fromRadians(turnEncoder.getPosition());
        inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
        inputs.turnAppliedVolts = turnMotor.getBusVoltage();
        inputs.turnCurrentAmps = turnMotor.getOutputCurrent();
    }

    /** Run the drive motor at the specified voltage. */
    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    /** Run the turn motor at the specified voltage. */
    @Override
    public void setTurnVoltage(double volts) {
        turnMotor.setVoltage(volts);
    }

    /** Enable or disable brake mode on the drive motor. */
    @Override
    public void setDriveBrakeMode(boolean enable) {
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    /** Enable or disable brake mode on the turn motor. */
    @Override
    public void setTurnBrakeMode(boolean enable) {
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }
}

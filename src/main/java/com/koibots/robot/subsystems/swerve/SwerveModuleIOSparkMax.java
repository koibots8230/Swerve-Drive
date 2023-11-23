package com.koibots.robot.subsystems.swerve;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {
    CANSparkMax driveMotor;
    CANSparkMax turnMotor;
    RelativeEncoder driveEncoder;
    RelativeEncoder turnEncoder;
    SparkMaxPIDController turnController;


    public SwerveModuleIOSparkMax(int driveID, int turnID) {
        driveMotor = new CANSparkMax(driveID, CANSparkMaxLowLevel.MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnID, CANSparkMaxLowLevel.MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
        // TODO: Double check through bore actual values

        SimpleMotorFeedforward x = new SimpleMotorFeedforward(0, 0, 0);
    }

    /** Updates the set of loggable inputs. */
    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        inputs.drivePositionRad = driveEncoder.getPosition();
        inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
        inputs.driveAppliedVolts = driveMotor.getBusVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();

        inputs.modulePosition = Rotation2d.fromRadians(turnEncoder.getPosition());
        inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
        inputs.turnAppliedVolts = turnMotor.getBusVoltage();
        inputs.turnCurrentAmps = turnMotor.getOutputCurrent();
    }

    /** Run the drive motor at the specified voltage. */
    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(MathUtil.clamp(volts, -11.5, 11.5));
    }

    /** Turn the module to an angle */
    @Override
    public void setModuleAngle(double radians) {

    }

    @Override
    public void stopTurnMotor() {
        turnMotor.setVoltage(0);
    }

    /** Enable or disable brake mode on the drive motor. */
    @Override
    public void setDriveIdleMode(CANSparkMax.IdleMode mode) {
        driveMotor.setIdleMode(mode);
    }

    /** Enable or disable brake mode on the turn motor. */
    @Override
    public void setTurnIdleMode(CANSparkMax.IdleMode mode) {
        turnMotor.setIdleMode(mode);
    }
}

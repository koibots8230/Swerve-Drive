package com.koibots.robot.subsystems.swerve;

import com.koibots.robot.constants.Constants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;

public class SwerveModuleIOMaxSwerve implements SwerveModuleIO {
    CANSparkMax turningSparkMax;
    CANSparkMax drivingSparkMax;

    private final RelativeEncoder drivingEncoder;
    private final AbsoluteEncoder azimuthEncoder;

    private final SparkMaxPIDController drivingPIDController;
    private final SparkMaxPIDController azimuthPIDController;

    private final double chassisAngularOffset;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    protected SwerveModuleIOMaxSwerve(
            int drivingCANId,
            int azimuthCANId,
            double chassisAngularOffset
    ) {
        try {
            turningSparkMax = new CANSparkMax(azimuthCANId, CANSparkMaxLowLevel.MotorType.kBrushless);
            drivingSparkMax = new CANSparkMax(drivingCANId, CANSparkMaxLowLevel.MotorType.kBrushless);

            drivingSparkMax.restoreFactoryDefaults();
            turningSparkMax.restoreFactoryDefaults();

            // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
            drivingEncoder = drivingSparkMax.getEncoder();
            azimuthEncoder = turningSparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
            drivingPIDController = drivingSparkMax.getPIDController();
            azimuthPIDController = turningSparkMax.getPIDController();
            drivingPIDController.setFeedbackDevice(drivingEncoder);
            azimuthPIDController.setFeedbackDevice(azimuthEncoder);

            // Apply position and velocity conversion factors for the driving encoder. The
            // native units for position and velocity are rotations and RPM, respectively,
            // but we want meters and meters per second to use with WPILib's swerve APIs.
            drivingEncoder.setPositionConversionFactor(Constants.ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR);
            drivingEncoder.setVelocityConversionFactor(Constants.ModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR);

            // Apply position and velocity conversion factors for the turning encoder. We
            // want these in radians and radians per second to use with WPILib's swerve
            // APIs.
            azimuthEncoder.setPositionConversionFactor(Constants.ModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
            azimuthEncoder.setVelocityConversionFactor(Constants.ModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);

            // Invert the turning encoder, since the output shaft rotates in the opposite direction of
            // the steering motor in the MAXSwerve Module.
            azimuthEncoder.setInverted(Constants.ModuleConstants.TURNING_ENCODER_INVERTED);


            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction, which is a
            // longer route.
            azimuthPIDController.setPositionPIDWrappingEnabled(true);
            azimuthPIDController.setPositionPIDWrappingMinInput(Constants.ModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT);
            azimuthPIDController.setPositionPIDWrappingMaxInput(Constants.ModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT);

            // Set the PID gains for the driving motor. Note these are example gains, and you
            // may need to tune them for your own robot!
            drivingPIDController.setP(Constants.ModuleConstants.DRIVING_P);
            drivingPIDController.setI(Constants.ModuleConstants.DRIVING_I);
            drivingPIDController.setD(Constants.ModuleConstants.DRIVING_D);
            drivingPIDController.setFF(Constants.ModuleConstants.DRIVING_FF);
            drivingPIDController.setOutputRange(Constants.ModuleConstants.DRIVING_MIN_OUTPUT,
                    Constants.ModuleConstants.DRIVING_MAX_OUTPUT);

            // Set the PID gains for the turning motor. Note these are example gains, and you
            // may need to tune them for your own robot!
            azimuthPIDController.setP(Constants.ModuleConstants.TURNING_P);
            azimuthPIDController.setI(Constants.ModuleConstants.TURNING_I);
            azimuthPIDController.setD(Constants.ModuleConstants.TURNING_D);
            azimuthPIDController.setFF(Constants.ModuleConstants.TURNING_FF);
            azimuthPIDController.setOutputRange(Constants.ModuleConstants.TURNING_MIN_OUTPUT,
                    Constants.ModuleConstants.TURNING_MAX_OUTPUT);

            drivingSparkMax.setIdleMode(Constants.ModuleConstants.DRIVING_MOTOR_IDLE_MODE);
            turningSparkMax.setIdleMode(Constants.ModuleConstants.TURNING_MOTOR_IDLE_MODE);
            drivingSparkMax.setSmartCurrentLimit(Constants.ModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT);
            turningSparkMax.setSmartCurrentLimit(Constants.ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);

            // Save the SPARK MAX configurations. If a SPARK MAX browns out during
            // operation, it will maintain the above configurations.
            drivingSparkMax.burnFlash();
            turningSparkMax.burnFlash();

            this.chassisAngularOffset = chassisAngularOffset;
            desiredState.angle = new Rotation2d(azimuthEncoder.getPosition());
            drivingEncoder.setPosition(0);
        } catch (RuntimeException e) {
            DriverStation.reportError("Failed instantiate swerve module", e.getStackTrace());
            throw e;
        }
    }

    @Override
    public void updateState(SwerveModuleIOState state) {
        state.angle = azimuthEncoder.getPosition() - chassisAngularOffset;
        state.distance = drivingEncoder.getPosition();
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                new Rotation2d(azimuthEncoder.getPosition()));

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        azimuthPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        this.desiredState = desiredState;
    }

    @Override
    public void resetEncoders() {
        drivingEncoder.setPosition(0);
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                this.drivingEncoder.getPosition(),
                new Rotation2d(this.azimuthEncoder.getPosition())
        );
    }

    @Override
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
                this.drivingEncoder.getVelocity(),
                new Rotation2d(this.azimuthEncoder.getPosition())
        );
    }

}

package com.koibots.robot.subsystems.swerve;

import com.koibots.lib.hardware.NavX;
import com.koibots.robot.Robot;
import com.koibots.robot.constants.Constants.DriveConstants;
import com.koibots.robot.constants.ControlConstants;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static com.koibots.robot.constants.Constants.DriveConstants.DRIVE_KINEMATICS;
import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;
import static edu.wpi.first.math.kinematics.SwerveDriveKinematics.desaturateWheelSpeeds;

public class Swerve extends SubsystemBase {
    private final SwerveModuleIO frontLeft;
    private final SwerveModuleIO frontRight;
    private final SwerveModuleIO rearLeft;
    private final SwerveModuleIO rearRight;

    private final SwerveModuleIOStateAutoLogged frontLeftState = new SwerveModuleIOStateAutoLogged();
    private final SwerveModuleIOStateAutoLogged frontRightState = new SwerveModuleIOStateAutoLogged();
    private final SwerveModuleIOStateAutoLogged rearLeftState = new SwerveModuleIOStateAutoLogged();
    private final SwerveModuleIOStateAutoLogged rearRightState = new SwerveModuleIOStateAutoLogged();
    private double odometryTimeOffset;
    private final SwerveDrivePoseEstimator odometry;
    public final HolonomicDriveController trajectoryFollower = new HolonomicDriveController(
            ControlConstants.xController,
            ControlConstants.yController,
            ControlConstants.thetaController
    );

    SimDouble gyroYaw;

    /**
     * <p>Forward is 0 heading</p>
     *
     * @author CalebNeal07
     */
    public Swerve() {
        if (Robot.isReal()) {
            frontLeft = new SwerveModuleIOMaxSwerve(
                    DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
                    DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
                    DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

            frontRight = new SwerveModuleIOMaxSwerve(
                    DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
                    DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
                    DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

            rearLeft = new SwerveModuleIOMaxSwerve(
                    DriveConstants.REAR_LEFT_DRIVING_CAN_ID,
                    DriveConstants.REAR_LEFT_TURNING_CAN_ID,
                    DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

            rearRight = new SwerveModuleIOMaxSwerve(
                    DriveConstants.REAR_RIGHT_DRIVING_CAN_ID,
                    DriveConstants.REAR_RIGHT_TURNING_CAN_ID,
                    DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);
        } else {
            frontLeft = new SwerveModuleIOSim(
                    DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
                    DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
                    DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

            frontRight = new SwerveModuleIOSim(
                    DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
                    DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
                    DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

            rearLeft = new SwerveModuleIOSim(
                    DriveConstants.REAR_LEFT_DRIVING_CAN_ID,
                    DriveConstants.REAR_LEFT_TURNING_CAN_ID,
                    DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

            rearRight = new SwerveModuleIOSim(
                    DriveConstants.REAR_RIGHT_DRIVING_CAN_ID,
                    DriveConstants.REAR_RIGHT_TURNING_CAN_ID,
                    DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

            gyroYaw = new SimDouble(
                    SimDeviceDataJNI.getSimValueHandle(
                            SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"),
                            "Yaw"
                    )
            );
        }

        odometry = new SwerveDrivePoseEstimator(
                DRIVE_KINEMATICS,
                fromDegrees(NavX.get().getAngle()),
                new SwerveModulePosition[] {
                        frontLeft.getModulePosition(),
                        frontRight.getModulePosition(),
                        rearLeft.getModulePosition(),
                        rearRight.getModulePosition()
                },
                new Pose2d()
        );


        odometryTimeOffset = Timer.getFPGATimestamp();

        try(
                Notifier odometryUpdater = new Notifier(() ->
                        odometry.updateWithTime(
                                Timer.getFPGATimestamp() - odometryTimeOffset,
                                fromDegrees(NavX.get().getAngle()),
                                new SwerveModulePosition[]{
                                        frontLeft.getModulePosition(),
                                        frontRight.getModulePosition(),
                                        rearLeft.getModulePosition(),
                                        rearRight.getModulePosition()
                                }
                        )
                )
        ) {
            odometryUpdater.startPeriodic(1.0f / 250.0f); // Start odometry with frequency of 250hz
        } catch (Exception e) {
            DriverStation.reportError("Failed to initialize odometry thread", false);
        }

        trajectoryFollower.setTolerance(new Pose2d(new Translation2d(0.1, 0.1), new Rotation2d(0.1)));
    }

    @Override
    public void periodic() {
        frontLeft.updateState(frontLeftState);
        frontRight.updateState(frontRightState);
        rearLeft.updateState(rearLeftState);
        rearRight.updateState(rearRightState);

        Logger.getInstance().processInputs("Front Left Swerve Module", frontLeftState);
        Logger.getInstance().processInputs("Front Right Swerve Module", frontRightState);
        Logger.getInstance().processInputs("Rear Left Swerve Module", rearLeftState);
        Logger.getInstance().processInputs("Rear Right Swerve Module", rearRightState);
    }

    @Override
    public void simulationPeriodic() {
        gyroYaw.set(odometry.getEstimatedPosition().getRotation().getDegrees());

        Logger.getInstance().recordOutput("Sim/Odometry", odometry.getEstimatedPosition());
        Logger.getInstance().recordOutput("Sim/FrontLeftModule", frontLeft.getModuleState(),
                frontRight.getModuleState(),
                rearLeft.getModuleState(),
                rearRight.getModuleState());
    }

    public Pose2d getEstimatedPosition() {
        return odometry.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        odometryTimeOffset = Timer.getFPGATimestamp();

        odometry.resetPosition(fromDegrees(NavX.get().getAngle()),
                new SwerveModulePosition[]{
                        frontLeft.getModulePosition(),
                        frontRight.getModulePosition(),
                        rearLeft.getModulePosition(),
                        rearRight.getModulePosition()
                },
                pose
        );
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        rearLeft.resetEncoders();
        rearRight.resetEncoders();
    }

    public void setCross() {
        frontLeft.setDesiredState(new SwerveModuleState(0, fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, fromDegrees(-45)));
        rearLeft.setDesiredState(new SwerveModuleState(0, fromDegrees(-45)));
        rearRight.setDesiredState(new SwerveModuleState(0, fromDegrees(45)));
    }

    public void stop() {
        frontLeft.setDesiredState(
                new SwerveModuleState(0, frontLeft.getModulePosition().angle)
        );

        frontRight.setDesiredState(
                new SwerveModuleState(0, frontRight.getModulePosition().angle)
        );

        rearLeft.setDesiredState(
                new SwerveModuleState(0, rearLeft.getModulePosition().angle)
        );

        rearRight.setDesiredState(
                new SwerveModuleState(0, rearRight.getModulePosition().angle)
        );
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        desaturateWheelSpeeds(
                desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);
    }

    public void addVisionMeasurement(Pose2d pose, long time) {
        this.odometry.addVisionMeasurement(pose, time);
    }
}

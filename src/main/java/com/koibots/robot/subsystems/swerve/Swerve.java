package com.koibots.robot.subsystems.swerve;

import com.koibots.lib.hardware.NavX;
import com.koibots.robot.Robot;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.koibots.robot.constants.Constants;
import com.koibots.robot.constants.Constants.DriveConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static com.koibots.robot.constants.Constants.DriveConstants.DRIVE_KINEMATICS;
import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;
import static edu.wpi.first.math.kinematics.SwerveDriveKinematics.desaturateWheelSpeeds;
import static java.lang.StrictMath.PI;

public class Swerve extends SubsystemBase {
    private final SwerveModuleIO frontLeft;
    private final SwerveModuleIO frontRight;
    private final SwerveModuleIO rearLeft;
    private final SwerveModuleIO rearRight;

    private final SwerveModuleIOStateAutoLogged frontLeftState = new SwerveModuleIOStateAutoLogged();
    private final SwerveModuleIOStateAutoLogged frontRightState = new SwerveModuleIOStateAutoLogged();
    private final SwerveModuleIOStateAutoLogged rearLeftState = new SwerveModuleIOStateAutoLogged();
    private final SwerveModuleIOStateAutoLogged rearRightState = new SwerveModuleIOStateAutoLogged();

    private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
            DRIVE_KINEMATICS,
            fromDegrees(NavX.getInstance().getAngle()),
            new SwerveModulePosition[] {
                    frontLeftState.getModulePosition(),
                    frontRightState.getModulePosition(),
                    rearLeftState.getModulePosition(),
                    rearRightState.getModulePosition()
            },
            new Pose2d()
    );

    /**
     * <p>Forward is 0 heading</p>
     *
     * @author Caleb O'Neal
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
            frontLeft = new SwerveModuleIOSim();
            frontRight = new SwerveModuleIOSim();
            rearLeft = new SwerveModuleIOSim();
            rearRight = new SwerveModuleIOSim();
        }
    }

    @Override
    public void periodic() {
        frontLeft.updateState(frontLeftState);
        frontRight.updateState(frontRightState);
        rearLeft.updateState(rearLeftState);
        rearRight.updateState(rearRightState);

        odometry.update(
                fromDegrees(NavX.getInstance().getAngle()),
                new SwerveModulePosition[]{
                        frontLeftState.getModulePosition(),
                        frontRightState.getModulePosition(),
                        rearLeftState.getModulePosition(),
                        rearRightState.getModulePosition()
                }
        );

        Logger.getInstance().processInputs("Front Left Swerve Module", frontLeftState);
        Logger.getInstance().processInputs("Front Right Swerve Module", frontRightState);
        Logger.getInstance().processInputs("Rear Left Swerve Module", rearLeftState);
        Logger.getInstance().processInputs("Rear Right Swerve Module", rearRightState);
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(fromDegrees(NavX.getInstance().getAngle()),
                new SwerveModulePosition[]{
                        frontLeftState.getModulePosition(),
                        frontRightState.getModulePosition(),
                        rearLeftState.getModulePosition(),
                        rearRightState.getModulePosition()
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

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        desaturateWheelSpeeds(
                desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);
    }

    public class FieldOrientedDrive extends CommandBase {
        private final DoubleSupplier vxSupplier;
        private final DoubleSupplier vySupplier;
        private final DoubleSupplier angularVelocitySupplier;

        public FieldOrientedDrive(
                DoubleSupplier vxSupplier,
                DoubleSupplier vySupplier,
                DoubleSupplier angularVelocitySupplier
        ) {
            this.vxSupplier = vxSupplier;
            this.vySupplier = vySupplier;
            this.angularVelocitySupplier = angularVelocitySupplier;

            addRequirements(Swerve.this);
        }

        @Override
        public void execute() {
            var setpointTwist = 
                new Pose2d()
                    .log(
                        new Pose2d(
                            vxSupplier.getAsDouble() * Constants.PERIOD,
                            vySupplier.getAsDouble() * Constants.PERIOD,
                            new Rotation2d(angularVelocitySupplier.getAsDouble() * Constants.DriveConstants.MAX_ANGULAR_SPEED * Constants.PERIOD)));
            var adjustedSpeeds = 
                new ChassisSpeeds(
                    setpointTwist.dx / Constants.PERIOD,
                    setpointTwist.dy / Constants.PERIOD,
                    setpointTwist.dtheta / Constants.PERIOD);    

            var targetModuleStates = DRIVE_KINEMATICS.toSwerveModuleStates(adjustedSpeeds);

            desaturateWheelSpeeds(targetModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
            setModuleStates(targetModuleStates);
        }

        @Override
        public void end(boolean interrupted) {
            DriverStation.reportError("Swerve default command was cancelled", false);
        }
    }
}

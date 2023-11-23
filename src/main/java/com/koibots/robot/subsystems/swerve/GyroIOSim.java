package com.koibots.robot.subsystems.swerve;

import com.koibots.robot.Constants.DrivetrainConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class GyroIOSim implements GyroIO {
    boolean zero = false;
    Field2d field = new Field2d();

    public GyroIOSim() {
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        field.setRobotPose(Swerve.get().getEstimatedPose());

        ChassisSpeeds speeds = DrivetrainConstants.SWERVE_KINEMATICS.toChassisSpeeds(Swerve.get().getModuleStates());
        inputs.yawPosition = inputs.yawPosition
                .plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * 0.02));
        inputs.yawVelocityRadPerSec = RadiansPerSecond.of(speeds.omegaRadiansPerSecond);

        if (zero) {
            inputs.yawPosition = new Rotation2d();
            zero = false;
        }
    }

    @Override
    public void zeroYaw() {
        zero = true;
    }
}

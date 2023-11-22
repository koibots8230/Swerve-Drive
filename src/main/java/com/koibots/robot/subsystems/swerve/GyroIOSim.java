package com.koibots.robot.subsystems.swerve;

import com.koibots.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class GyroIOSim implements GyroIO {
    boolean zero = false;

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        ChassisSpeeds speeds = Constants.SWERVE_KINEMATICS.toChassisSpeeds(Swerve.get().getModuleStates());

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

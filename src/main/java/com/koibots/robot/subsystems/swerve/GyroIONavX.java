package com.koibots.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class GyroIONavX implements GyroIO {
    AHRS gyro;

    protected GyroIONavX() {
        gyro = new AHRS(Port.kMXP);

        gyro.zeroYaw();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        Rotation2d previousYaw = inputs.yawPosition;

        inputs.yawPosition = Rotation2d.fromDegrees(-gyro.getYaw());

        inputs.yawVelocityRadPerSec = RadiansPerSecond.of(inputs.yawPosition.minus(previousYaw).getRadians() / 0.02);
    }

    @Override
    public void zeroYaw() {
        gyro.zeroYaw();
    }
}

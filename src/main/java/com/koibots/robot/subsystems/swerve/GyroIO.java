package com.koibots.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public Rotation2d yawPosition = new Rotation2d();
        public Measure<Velocity<Angle>> yawVelocityRadPerSec = RadiansPerSecond.of(0);
    }

    public void updateInputs(GyroIOInputs inputs);

    public void zeroYaw();
}

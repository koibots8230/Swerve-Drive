// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package com.koibots.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public Rotation2d modulePosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    void updateInputs(SwerveModuleInputs inputs);

    /** Run the drive motor at the specified voltage. */
    void setDriveVoltage(double volts);
    void setModuleAngle(double radians);
    void stopTurnMotor();

    /** Enable or disable brake mode on the drive motor. */
    default void setDriveIdleMode(CANSparkMax.IdleMode mode) {}

    /** Enable or disable brake mode on the turn motor. */
    default void setTurnIdleMode(CANSparkMax.IdleMode mode) {}
}

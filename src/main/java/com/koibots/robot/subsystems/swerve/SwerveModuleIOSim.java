package com.koibots.robot.subsystems.swerve;

import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleIOSim implements SwerveModuleIO {


    protected SwerveModuleIOSim() {

    }

    @Override
    public void updateState(SwerveModuleIOState state) {

    }

    @Override
    public void setDesiredState(SwerveModuleState state) {

    }

    @Override
    public void resetEncoders() {

    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return null;
    }
}

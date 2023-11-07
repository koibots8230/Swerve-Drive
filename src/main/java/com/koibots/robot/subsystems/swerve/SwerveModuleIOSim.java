package com.koibots.robot.subsystems.swerve;

import com.koibots.lib.debug.DebugUtils;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;

public class SwerveModuleIOSim extends SwerveModuleIOMaxSwerve {

    protected SwerveModuleIOSim(int drivingCANId, int azimuthCANId, double chassisAngularOffset) {
        super(drivingCANId, azimuthCANId, chassisAngularOffset);

        REVPhysicsSim.getInstance().addSparkMax(
                super.drivingSparkMax, DCMotor.getNEO(1)
        );

        REVPhysicsSim.getInstance().addSparkMax(
                super.turningSparkMax, DCMotor.getNEO(1)
        );
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        super.setDesiredState(desiredState);

        System.out.println(desiredState);
    }
}

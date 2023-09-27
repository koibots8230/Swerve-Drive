package com.koibots.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO  {
    @AutoLog
    class SwerveIOState {
        
    }
    
    void updateState(SwerveIOState state);

}
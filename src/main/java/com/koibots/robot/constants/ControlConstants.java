package com.koibots.robot.constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ControlConstants {
    public static final PIDController xController = new PIDController(0.3, 0, 0);
    public static final PIDController yController = new PIDController(0.3, 0, 0);
    public static final ProfiledPIDController thetaController = new ProfiledPIDController(
            0.2,
            0,
            0,
            new TrapezoidProfile.Constraints(Constants.DriveConstants.MAX_ANGULAR_SPEED, Math.PI)
    );
}

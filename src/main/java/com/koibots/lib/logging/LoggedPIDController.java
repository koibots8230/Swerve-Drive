package com.koibots.lib.logging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LoggedPIDController extends PIDController {
    String name;

    public LoggedPIDController(double kp, double ki, double kd, String name) {
        this(kp, ki, kd, 0.02, name);
    }

    public LoggedPIDController(double kp, double ki, double kd, double period, String name) {
        super(kp, ki, kd, period);

        SmartDashboard.putData(name, this);

    }

}

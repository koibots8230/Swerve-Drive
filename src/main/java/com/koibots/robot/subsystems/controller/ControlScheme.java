package com.koibots.robot.subsystems.controller;

import org.littletonrobotics.junction.AutoLog;

public abstract class ControlScheme {

    @AutoLog
    class ControllerIO {
        double xTranslation;
        double yTranslation;

    }

    public abstract double xTranslation();

    public abstract double yTranslation();

    public abstract double angularVelocity();

    public abstract int anglePosition();

    public abstract boolean cross();
}

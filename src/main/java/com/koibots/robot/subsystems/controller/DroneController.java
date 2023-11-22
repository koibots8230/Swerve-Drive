package com.koibots.robot.subsystems.controller;

public class DroneController extends ControlScheme {

    @Override
    public double xTranslation() {
        return 0;
    }

    @Override
    public double yTranslation() {
        return 0;
    }

    @Override
    public double angularVelocity() {
        return 0;
    }

    @Override
    public int anglePosition() {
        return 0;
    }

    @Override
    public boolean cross() {
        return false;
    }
}

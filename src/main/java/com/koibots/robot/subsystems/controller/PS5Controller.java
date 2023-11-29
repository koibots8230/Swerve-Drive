package com.koibots.robot.subsystems.controller;

public class PS5Controller extends ControlScheme {

    edu.wpi.first.wpilibj.PS4Controller controller;

    public PS5Controller() {
        controller = new edu.wpi.first.wpilibj.PS4Controller(0);
    }

    @Override
    public double xTranslation() {
        return -controller.getLeftY();
    }

    @Override
    public double yTranslation() {
        return -controller.getLeftX();
    }

    @Override
    public double angularVelocity() {
        return -controller.getRawAxis(2);
    }

    @Override
    public int anglePosition() {
        return controller.getPOV();
    }

    @Override
    public boolean cross() {
        return controller.getRawButton(2);
    }

}
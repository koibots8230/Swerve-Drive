package com.koibots.robot.subsystems.controller;

public class XboxController extends ControlScheme {
    private final edu.wpi.first.wpilibj.XboxController controller;

    public XboxController() {
        controller = new edu.wpi.first.wpilibj.XboxController(0);
    }

    @Override
    public double xTranslation() {
        return 0;
    }

    public double yTranslation() {
        return 0;
    }

    public double angularVelocity() {
        return 0;
    }


    public int anglePosition() {
        return 0;
    }

    public boolean cross() {
        return false;
    }

}

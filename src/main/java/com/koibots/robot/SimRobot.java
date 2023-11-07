package com.koibots.robot;

import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

public class SimRobot {
    private static final SimRobot instance;

    static {
        instance = new SimRobot();
    }

    public static SimRobot getInstance() {
        return instance;
    }

    Field2d field;

    SimRobot() {
        field = new Field2d();
    }

    public void run() {
        REVPhysicsSim.getInstance().run();
        field.setRobotPose(Swerve.get().getEstimatedPosition());
    }


}

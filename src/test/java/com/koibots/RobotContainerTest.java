package com.koibots;

import org.junit.jupiter.api.Test;

import com.koibots.robot.RobotContainer;
import com.koibots.robot.Robot.Mode;

public class RobotContainerTest {
    @Test
    public void instantiateRobotContainer() {
        new RobotContainer(Mode.SIM);
    }
}

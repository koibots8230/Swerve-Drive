package com.koibots.robot.subsystems;

import com.koibots.robot.subsystems.swerve.Swerve;
import com.koibots.robot.subsystems.vision.Vision;

import java.util.function.Supplier;

public class Subsystems {
    private static Swerve swerveInstance;
    public static Supplier<Swerve> Swerve;

    private static Vision visionInstance;
    public static Supplier<Vision> Vision;

    static {
      Swerve = () -> {
          swerveInstance = new Swerve();
          Swerve = () -> {
              return swerveInstance;
          };

          return swerveInstance;
      };

      Vision = () -> {
          visionInstance = new Vision();
          Vision = () -> {
              return visionInstance;
          };

          return visionInstance;
      };
    }
}

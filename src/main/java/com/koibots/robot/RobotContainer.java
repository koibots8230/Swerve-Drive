// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.koibots.robot;

import com.koibots.lib.math.SwerveUtils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import static com.koibots.robot.constants.Constants.OIConstants;
import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.math.MathUtil.applyDeadband;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The driver's controller
  XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);

  PIDController robotRotationController = new PIDController( // TODO: Find real constants
          0.2,
          0,
          0,
          0.02
  );

  LoggedDashboardChooser<Path> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
      /*
      try (var autos = Files.list(Filesystem.getDeployDirectory().toPath());) {
          autos.forEach(
                  (auto) ->  autoChooser.addOption(
                          pathToAutoName(auto),
                          auto
                  )
          );
      } catch (IOException e) {
          DriverStation.reportError("Failed to access deploy directory", false);
      } finally {
          autoChooser.addDefaultOption("None", null);
      }*/

      robotRotationController.enableContinuousInput(0, 360); // Allows PID loop to wrap at 360 degrees

      // Configure the button bindings
      configureButtonBindings();

      // Configure default commands
      Swerve.get().setDefaultCommand(
              Swerve.get().new FieldOrientedDrive(
                      () -> applyDeadband(driverController.getLeftX(), 0.02),
                      () -> applyDeadband(driverController.getLeftY(), 0.02),
                      () -> {
                          if (driverController.getPOV() != -1) {
                              return applyDeadband(driverController.getRightX(), 0.02);
                          } else {
                              return SwerveUtils.wrapAngle(robotRotationController.calculate(driverController.getPOV()));
                          }
                      }
              )
      );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, Button.kR1.value)
            .whileTrue(new RunCommand(
                    Swerve.get()::setCross,
                    Swerve.get()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      return null;
    // return new SwerveAutonomousController(autoChooser.get());
  }
}
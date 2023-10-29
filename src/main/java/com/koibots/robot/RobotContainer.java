package com.koibots.robot;

import com.koibots.lib.math.SwerveUtils;
import com.koibots.robot.command.FieldOrientedDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

    PIDController robotRotationController = new PIDController(
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

       // Allows PID loop to wrap at 360 degrees
        robotRotationController.enableContinuousInput(0, 360);

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
    public void configureButtonBindings() {
        enableFieldOrientedControl();

        new JoystickButton(driverController, Button.kR1.value)
            .whileTrue(new RunCommand(
                    Swerve.get()::setCross,
                    Swerve.get()
            ).finallyDo(
                    (_end) -> enableFieldOrientedControl()
            ));
    }

    private void enableFieldOrientedControl() {
      Swerve.get().setDefaultCommand(
              new FieldOrientedDrive(
                      () -> applyDeadband(driverController.getLeftX(), 0.02),
                      () -> applyDeadband(driverController.getLeftY(), 0.02),
                      () -> {
                          if (driverController.getPOV() != -1) {
                              return applyDeadband(driverController.getRightX(), 0.02);
                          } else {
                              return MathUtil.clamp(
                                      SwerveUtils.wrapAngle(robotRotationController.calculate(
                                              Swerve.get().getEstimatedPosition().getRotation().getDegrees(),
                                              driverController.getPOV()
                                      )),
                                      -1,
                                      1
                              );
                          }
                      }
              )
      );
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
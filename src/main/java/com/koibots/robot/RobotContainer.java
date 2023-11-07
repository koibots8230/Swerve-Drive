package com.koibots.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.nio.file.Path;

import static com.koibots.robot.subsystems.Subsystems.Swerve;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Controllers
    private final CommandXboxController xbox = new CommandXboxController(0);
    private final CommandJoystick joystick = new CommandJoystick(0);
    private final CommandGenericHID drone = new CommandGenericHID(0);

    enum Controller {
        Xbox,
        Joystick,
        DroneController
    }

    LoggedDashboardChooser<Controller> controllerChooser;

    /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
    public RobotContainer(Robot.Mode mode) {
        switch (mode) {
            case REPLAY:
                DriverStation.reportError("Replay not supported", false);
                throw new RuntimeException("Replay not supported");
            case REAL:
            case SIM:
                controllerChooser.addDefaultOption("Xbox Controller", Controller.Xbox);
                controllerChooser.addOption("Flight Joystick", Controller.Joystick);
                controllerChooser.addOption("Drone Controller", Controller.DroneController);

                configureButtonBindings();
                break;
        }
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
        switch (controllerChooser.get()) {
            case Xbox:

                break;
            case Joystick:

                break;
            case DroneController:

                break;
        }
    }

    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        return null;
    }
}
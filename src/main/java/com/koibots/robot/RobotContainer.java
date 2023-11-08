package com.koibots.robot;

import com.koibots.robot.command.teleop.SwerveCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

import java.util.function.Function;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Controllers
    private final CommandXboxController xbox = new CommandXboxController(0);
    private final CommandPS5Controller ps5 = new CommandPS5Controller(0);
    private final CommandJoystick joystick = new CommandJoystick(0);
    private final CommandGenericHID drone = new CommandGenericHID(0);

    enum Controller {
        Xbox,
        PS5,
        Joystick,
        DroneController
    }

    LoggedDashboardChooser<Controller> controllerChooser;


    // Graph of algorithms here: https://www.desmos.com/calculator/w738aldioj
    enum ScalingAlgorithm {
        Linear(
                (x) -> x
        ),
        Squared(
                (x) -> Math.signum(x) * x * x
        ),

        Cubed(
                (x) -> x * x * x
        ),

        Cosine(
                (x) -> (-Math.signum(x) * Math.cos(Math.PI * 0.5 * x )) + (1 * Math.signum(x))
        ),

        CubedSquareRoot(
                (x) -> Math.signum(x) * Math.sqrt(Math.abs(x * x * x))
        );

        public final Function<Double, Double> algorithm;

        private ScalingAlgorithm(Function<Double, Double> algorithm) {
            this.algorithm = algorithm;
        }
    }


    LoggedDashboardChooser<ScalingAlgorithm> scalingChooser = new LoggedDashboardChooser<>("Scaling Algorithm");

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
                controllerChooser = new LoggedDashboardChooser<>("Controller Chooser");

                controllerChooser.addDefaultOption("Xbox Controller", Controller.Xbox);
                controllerChooser.addOption("PS5 Controller", Controller.PS5);
                controllerChooser.addOption("Flight Joystick", Controller.Joystick);
                controllerChooser.addOption("Drone Controller", Controller.DroneController);

                scalingChooser.addDefaultOption("Linear", ScalingAlgorithm.Linear);
                scalingChooser.addOption("Squared", ScalingAlgorithm.Squared);
                scalingChooser.addOption("Cubed", ScalingAlgorithm.Cubed);
                scalingChooser.addOption("Cosine", ScalingAlgorithm.Cosine);
                scalingChooser.addOption("Fancy", ScalingAlgorithm.CubedSquareRoot);

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
        SwerveCommand swerveCommand = switch (controllerChooser.get()) {
            case Xbox -> {
                System.out.println("Xbox Controller Initialized");
                yield new SwerveCommand(
                        () -> -xbox.getLeftY(),
                        () -> -xbox.getLeftX(),
                        xbox::getRightX,
                        () -> xbox.getHID().getPOV(),
                        () -> xbox.getHID().getAButton()
                );
            }
            case PS5 -> {
                System.out.println("PS5 Controller Initialized");
                yield new SwerveCommand(
                        () -> -ps5.getLeftY(),
                        () -> -ps5.getLeftX(),
                        () -> -ps5.getHID().getRawAxis(2),
                        () -> ps5.getHID().getPOV(),
                        () -> ps5.getHID().getCrossButton()
                );
            }
            case Joystick -> {
                System.out.println("PS5 Controller Initialized");
                yield new SwerveCommand(
                        joystick::getY,
                        joystick::getX,
                        joystick::getTwist,
                        () -> joystick.getHID().getPOV(),
                        () -> joystick.getHID().getTrigger()
                );
            }
            case DroneController -> {
                System.out.println("Drone Controller Initialized");
                yield new SwerveCommand(
                        null,
                        null,
                        null,
                        null,
                        null
                );
            }
            default -> null;
        };

        swerveCommand.setScalingAlgorithm(scalingChooser.get().algorithm);
        Swerve.get().setDefaultCommand(swerveCommand);
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
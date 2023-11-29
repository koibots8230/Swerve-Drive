package com.koibots.robot;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

import com.koibots.lib.trajectory.Trajectory;
import com.koibots.robot.command.FieldOrientedDrive;
import com.koibots.robot.command.SwerveAutonomousController;
import com.koibots.robot.subsystems.controller.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.function.Function;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Graph of algorithms here: https://www.desmos.com/calculator/w738aldioj
    enum ScalingAlgorithm {
        Linear((x) -> x),
        Squared((x) -> Math.signum(x) * x * x),
        Cubed((x) -> x * x * x),
        Cosine((x) -> (-Math.signum(x) * Math.cos(Math.PI * 0.5 * x)) + (1 * Math.signum(x))),
        CubedSquareRoot((x) -> Math.signum(x) * Math.sqrt(Math.abs(x * x * x)));

        public final Function<Double, Double> algorithm;

        ScalingAlgorithm(Function<Double, Double> algorithm) {
            this.algorithm = algorithm;
        }
    }

    LoggedDashboardChooser<ScalingAlgorithm> scalingChooser;
    LoggedDashboardChooser<Supplier<ControlScheme>> controllerChooser;
    LoggedDashboardChooser<Command> autoChooser;

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
                scalingChooser = new LoggedDashboardChooser<>("Scaling Algorithm");
                autoChooser = new LoggedDashboardChooser<>("Auto Chooser");

                controllerChooser.addDefaultOption("PS5 Controller", PS5Controller::new);
                controllerChooser.addOption("Xbox Controller", XboxController::new);
                controllerChooser.addOption("Drone Controller", DroneController::new);
                controllerChooser.addOption("Flight Joystick", JoystickController::new);

                scalingChooser.addDefaultOption("Linear", ScalingAlgorithm.Linear);
                scalingChooser.addOption("Squared", ScalingAlgorithm.Squared);
                scalingChooser.addOption("Cubed", ScalingAlgorithm.Cubed);
                scalingChooser.addOption("Cosine", ScalingAlgorithm.Cosine);
                scalingChooser.addOption("Fancy", ScalingAlgorithm.CubedSquareRoot);

                autoChooser.addDefaultOption(
                        "Test", new SwerveAutonomousController(new Trajectory("Test"), true));

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
        ControlScheme controller = controllerChooser.get().get();

        Swerve.get().setDefaultCommand(
                new FieldOrientedDrive(
                        controller::xTranslation,
                        controller::yTranslation,
                        controller::angularVelocity,
                        controller::anglePosition,
                        controller::cross,
                        scalingChooser.get().algorithm));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}

package com.koibots.robot.command;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;

import java.io.File;
import java.io.FileNotFoundException;
import java.nio.file.Path;
import java.util.Scanner;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

public class SwerveAutonomousController extends PrintCommand {


    Scanner autoPath;
    LinearSystem x = LinearSystemId.identifyVelocitySystem()
    LinearQuadraticRegulator


    /**
     *
     *
     */
    public SwerveAutonomousController(Path autoRoutine) {

        super("Started Auto routine " + pathToAutoName(autoRoutine));

        try {
            this.autoPath = new Scanner(new File(autoRoutine.toString()));
        } catch (FileNotFoundException e) {
            DriverStation.reportError(
                    "Failed to find auto routine file " + pathToAutoName(autoRoutine),
                    false
            );
        }

        addRequirements(Swerve.get());
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    public static String pathToAutoName(Path auto) {
        var fileName = auto.getFileName().toString();

        // TODO Finish this function

        return "";
    }
}


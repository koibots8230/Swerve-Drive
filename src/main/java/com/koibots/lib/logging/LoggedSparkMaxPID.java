package com.koibots.lib.logging;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LoggedSparkMaxPID {
    SparkMaxPIDController pidController;
    int slot;
    SparkMaxPIDInputs inputs;

    protected LoggedSparkMaxPID(SparkMaxPIDController pidController, int slot) {
        this.pidController = pidController;
        this.slot = slot;
    }

    protected LoggedSparkMaxPID(SparkMaxPIDController pidController) {
        this.pidController = pidController;
    }

    public static class SparkMaxPIDInputs implements LoggableInputs {

        @Override
        public void toLog(LogTable table) {

        }

        @Override
        public void fromLog(LogTable table) {

        }
    }

    public void periodic() {

    }
}

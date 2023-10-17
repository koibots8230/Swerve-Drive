package com.koibots.lib.logging;

import com.revrobotics.CANSparkMax;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LoggedSparkMax {
    CANSparkMax sparkMax;
    SparkMaxInputs inputs = new SparkMaxInputs();

    public LoggedSparkMax(CANSparkMax sparkMax, String motorName) {
        this.sparkMax = sparkMax;
        inputs.deviceId = sparkMax.getDeviceId();
        inputs.firmware = sparkMax.getFirmwareString();
        inputs.firmwareVersion = sparkMax.getFirmwareVersion();
        periodic();
    }

    public static class SparkMaxInputs implements LoggableInputs {
        double speed = 0;
        double appliedOutput = 0;
        double busVoltage = 0;
        double closedLoopRampRate = 0;
        String idleMode = "";
        boolean isInverted = false;
        double temperature = 0;
        double openLoopRampRate = 0;
        double outputCurrent = 0;
        double voltageCompensationNominalVoltage = 0;
        int deviceId = 0;
        String firmware = "";
        int firmwareVersion = 0;

        @Override
        public void toLog(LogTable table) {

        }

        @Override
        public void fromLog(LogTable table) {

        }
    }

    public void periodic() {
        inputs.speed = sparkMax.get();
        inputs.appliedOutput = sparkMax.getAppliedOutput();
        inputs.busVoltage = sparkMax.getBusVoltage();
        inputs.closedLoopRampRate = sparkMax.getClosedLoopRampRate();
        inputs.idleMode = sparkMax.getIdleMode().toString();
        inputs.isInverted = sparkMax.getInverted();
        inputs.temperature = sparkMax.getMotorTemperature();
        inputs.openLoopRampRate = sparkMax.getOpenLoopRampRate();
        inputs.outputCurrent = sparkMax.getOutputCurrent();
        inputs.voltageCompensationNominalVoltage = sparkMax.getVoltageCompensationNominalVoltage();
    }
}

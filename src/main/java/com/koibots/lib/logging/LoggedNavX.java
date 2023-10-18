package com.koibots.lib.logging;

import com.koibots.lib.hardware.NavX;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LoggedNavX {
    private static final LoggedNavX instance;

    public static LoggedNavX get() {
        return instance;
    }

    static {
        instance = new LoggedNavX();
    }

    private final NavXInputs inputs = new NavXInputs();

    private LoggedNavX() {
        inputs.firmwareVersion = NavX.get().getFirmwareVersion();
    }

    public static class NavXInputs implements LoggableInputs {
        double altitude;
        double angleDegrees;
        int accelFullScaleRangeG;
        String firmwareVersion;
        double angleAdjustment;
        double actualUpdateRate;
        double barometricPressure;
        String boardYawAxis;
        float compassHeading;
        double displacementX;
        double displacementY;
        double displacementZ;
        double fusedHeading;
        double gyroFullScaleRangeDPS;
        long lastSensorTimestamp;
        double pitch;
        double pressure;
        double quaternionW;
        double quaternionX;
        double quaternionY;
        double quaternionZ;
        double rate;
        double rawAccelX;
        double rawAccelY;
        double rawAccelZ;
        double rawGyroX;
        double rawGyroY;
        double rawGyroZ;
        double rawMagX;
        double rawMagY;
        double rawMagZ;
        double requestedUpdateRate;
        double roll;
        double tempC;
        double updateCount;
        double velocityX;
        double velocityY;
        double velocityZ;
        double worldLinearAccelX;
        double worldLinearAccelY;
        double worldLinearAccelZ;
        double yaw;

        @Override
        public void toLog(LogTable table) {
            //TODO do these methods
        }

        @Override
        public void fromLog(LogTable table) {

        }
    }

    public void periodic() {
        inputs.altitude = NavX.get().getAltitude();
        inputs.angleDegrees = NavX.get().getAngle();
        inputs.accelFullScaleRangeG = NavX.get().getAccelFullScaleRangeG();
        inputs.angleAdjustment = NavX.get().getAngleAdjustment();
        inputs.actualUpdateRate = NavX.get().getActualUpdateRate();
        inputs.barometricPressure = NavX.get().getBarometricPressure();
        inputs.boardYawAxis = NavX.get().getBoardYawAxis().toString();
        inputs.compassHeading = NavX.get().getCompassHeading();
        inputs.displacementX = NavX.get().getDisplacementX();
        inputs.displacementY = NavX.get().getDisplacementY();
        inputs.displacementZ = NavX.get().getDisplacementZ();
        inputs.fusedHeading = NavX.get().getFusedHeading();
        inputs.gyroFullScaleRangeDPS = NavX.get().getGyroFullScaleRangeDPS();
        inputs.lastSensorTimestamp = NavX.get().getLastSensorTimestamp();
        inputs.pitch = NavX.get().getPitch();
        inputs.pressure = NavX.get().getPressure();
        inputs.quaternionW = NavX.get().getQuaternionW();
        inputs.quaternionX = NavX.get().getQuaternionX();
        inputs.quaternionY = NavX.get().getQuaternionY();
        inputs.quaternionZ = NavX.get().getQuaternionZ();
        inputs.rate = NavX.get().getRate();
        inputs.rawAccelX = NavX.get().getRawAccelX();
        inputs.rawAccelY = NavX.get().getRawAccelY();
        inputs.rawAccelZ = NavX.get().getRawAccelZ();
        inputs.rawGyroX = NavX.get().getRawGyroX();
        inputs.rawGyroY = NavX.get().getRawGyroY();
        inputs.rawGyroZ = NavX.get().getRawGyroZ();
        inputs.rawMagX = NavX.get().getRawMagX();
        inputs.rawMagY = NavX.get().getRawMagY();
        inputs.rawMagZ = NavX.get().getRawMagZ();
        inputs.requestedUpdateRate = NavX.get().getRequestedUpdateRate();
        inputs.roll = NavX.get().getRoll();
        inputs.tempC = NavX.get().getTempC();
        inputs.updateCount = NavX.get().getUpdateCount();
        inputs.velocityX = NavX.get().getVelocityX();
        inputs.velocityY = NavX.get().getVelocityY();
        inputs.velocityZ = NavX.get().getVelocityZ();
        inputs.worldLinearAccelX = NavX.get().getWorldLinearAccelX();
        inputs.worldLinearAccelY = NavX.get().getWorldLinearAccelY();
        inputs.worldLinearAccelZ = NavX.get().getWorldLinearAccelZ();
        inputs.yaw = NavX.get().getYaw();
    }

    public NavXInputs getInputs() {
        return inputs;
    }
}

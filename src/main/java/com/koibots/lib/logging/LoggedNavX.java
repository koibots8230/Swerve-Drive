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

    public static class NavXInputs implements LoggableInputs {
        double altitude = 0;


        @Override
        public void toLog(LogTable table) {

        }

        @Override
        public void fromLog(LogTable table) {

        }
    }

    public void periodic() {
        inputs. = NavX.get().getAltitude();
        inputs. = NavX.get().getAngle();
        inputs. = NavX.get().getAccelFullScaleRangeG();
        inputs. = NavX.get().getFirmwareVersion();
        inputs. = NavX.get().getAngleAdjustment();
        inputs. = NavX.get().getActualUpdateRate();
        inputs. = NavX.get().getBarometricPressure();
        inputs. = NavX.get().getBoardYawAxis();
        inputs. = NavX.get().getCompassHeading();
        inputs. = NavX.get().getDisplacementX();
        inputs. = NavX.get().getDisplacementY();
        inputs. = NavX.get().getDisplacementZ();
        inputs. = NavX.get().getFusedHeading();
        inputs. = NavX.get().getGyroFullScaleRangeDPS();
        inputs. = NavX.get().getLastSensorTimestamp();
        inputs. = NavX.get().getPitch();
        inputs. = NavX.get().getPressure();
        inputs. = NavX.get().getQuaternionW();
        inputs. = NavX.get().getQuaternionX();
        inputs. = NavX.get().getQuaternionY();
        inputs. = NavX.get().getQuaternionZ();
        inputs. = NavX.get().getRate();
        inputs. = NavX.get().getRawAccelX();
        inputs. = NavX.get().getRawAccelY();
        inputs. = NavX.get().getRawAccelZ();
        inputs. = NavX.get().getRawGyroX();
        inputs. = NavX.get().getRawGyroY();
        inputs. = NavX.get().getRawGyroZ();
        inputs. = NavX.get().getRawMagX();
        inputs. = NavX.get().getRawMagY();
        inputs. = NavX.get().getRawMagZ();
        inputs. = NavX.get().getRequestedUpdateRate();
        inputs. = NavX.get().getRoll();
        inputs. = NavX.get().getTempC();
        inputs. = NavX.get().getUpdateCount();
        inputs. = NavX.get().getVelocityX();
        inputs. = NavX.get().getVelocityY();
        inputs. = NavX.get().getVelocityZ();
        inputs. = NavX.get().getWorldLinearAccelX();
        inputs. = NavX.get().getWorldLinearAccelY();
        inputs. = NavX.get().getWorldLinearAccelZ();
        inputs. = NavX.get().getYaw();
    }

    public NavXInputs getInputs() {
        return inputs
    }
}

package com.koibots.lib.hardware;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;

public class NavX extends AHRS {
    private static final NavX instance;

    public static NavX getInstance() {
        return instance;
    }

    static {
        instance = new NavX();
    }

    private NavX() {
        super(SerialPort.Port.kMXP);
    }
}

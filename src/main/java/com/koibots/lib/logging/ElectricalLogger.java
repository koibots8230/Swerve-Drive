package com.koibots.lib.logging;

import edu.wpi.first.wpilibj.Notifier;

import java.util.ArrayList;

public class ElectricalLogger {
    private static final Notifier notifier;
    private static double period = 0.04; // 25Hz
    private static ArrayList<ElectricalLog> logPoints;

    static {
        notifier = new Notifier(() -> {
            logPoints.forEach(
                    ElectricalLog::logElectricalData
            );
        });
    }

    public static void addLogPoint(ElectricalLog log) {
        logPoints.add(log);
    }

    public static void setUpdateRate(int hz) {
        period = 1.0/hz;
    }

    public static void start() {
        notifier.startPeriodic(period);
    }
}

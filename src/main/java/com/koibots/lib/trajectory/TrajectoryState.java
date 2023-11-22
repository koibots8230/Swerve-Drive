package com.koibots.lib.trajectory;

public class TrajectoryState {
    public double timestamp;
    public double x;
    public double y;
    public double heading;
    public double velocityX;
    public double velocityY;
    public double angularVelocity;

    public TrajectoryState(
            double timestamp,
            double x,
            double y,
            double heading,
            double velocityX,
            double velocityY,
            double angularVelocity
    ) {
        this.timestamp = timestamp;
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.angularVelocity = angularVelocity;
    }
}

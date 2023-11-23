package com.koibots.lib.trajectory;

import com.koibots.lib.chargedup.FieldConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import lombok.Builder;

@Builder
public class TrajectoryState implements Interpolatable<TrajectoryState> {
    public final double timestamp;
    public final double x;
    public final double y;
    public final double heading;

    public final double velocityX;
    public final double velocityY;
    public final double angularVelocity;

    public Pose2d getPose() {
        return new Pose2d(x, y, Rotation2d.fromRadians(heading));
    }

    @Override
    public TrajectoryState interpolate(TrajectoryState endValue, double t) {
        double scale = (timestamp - t) / (endValue.timestamp - t);
        var interpolated_pose = getPose().interpolate(endValue.getPose(), scale);

        return TrajectoryState.builder()
                .x(interpolated_pose.getX())
                .y(interpolated_pose.getY())
                .heading(interpolated_pose.getRotation().getRadians())
                .velocityX(MathUtil.interpolate(this.velocityX, endValue.velocityX, scale))
                .velocityY(MathUtil.interpolate(this.velocityY, endValue.velocityY, scale))
                .angularVelocity(
                        MathUtil.interpolate(this.angularVelocity, endValue.angularVelocity, scale))
                .build();
    }

    public TrajectoryState flipped() {
        return TrajectoryState.builder()
                .x(FieldConstants.FIELD_WIDTH_METERS - this.x)
                .y(this.y)
                .heading(Math.PI - this.heading)
                .velocityX(this.velocityX * -1)
                .velocityY(this.velocityY)
                .angularVelocity(this.angularVelocity * -1)
                .build();
    }
}
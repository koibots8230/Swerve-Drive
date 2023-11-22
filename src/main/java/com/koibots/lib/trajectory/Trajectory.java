package com.koibots.lib.trajectory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.List;

public class Trajectory {
    List<TrajectoryState> states;

    /**
     * Loads a Choreo Trajectory
     * @param name the name of the file, no file extension needed.
     */
    public Trajectory(String name) {


        for (var point : points) {
            states.add(
                    new TrajectoryState(
                            point[0],
                            point[1],
                            point[2],
                            point[3],
                            point[4],
                            point[5],
                            point[6]
                    )
            )
        }
    }

    /**
     *
     * @param time the time in the trajectory
     * @return The speeds
     */

    public ChassisSpeeds step(double time) {

    }
}

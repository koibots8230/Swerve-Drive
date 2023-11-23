package com.koibots.lib.trajectory;

import com.google.gson.Gson;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.Arrays;
import java.util.List;

public class Trajectory {
    List<TrajectoryState> states;
    int step = 0;

    /**
     * Loads a Choreo Trajectory
     * @param name the name of the file, no file extension needed.
     */
    public Trajectory(String name) {
        Gson gson = new Gson();
        try (FileReader file = new FileReader(Filesystem.getDeployDirectory() + "choreo/" + name);) {
            states = Arrays.asList(gson.fromJson(new BufferedReader(file), TrajectoryState[].class));
        } catch (Exception e) {
            DriverStation.reportError("Failed to load Choreo Trajectory", e.getStackTrace());
        }
    }

    public Pose2d getInitialPose() {
        return states.get(0).getPose();
    }

    /**
     * Returns the trajectory state at the given time.
     * @param time the time in the trajectory
     * @return The speeds
     */
    public TrajectoryState sample(double time) {
        TrajectoryState state = states.get(step).interpolate(states.get(step + 1), time);
        step++;
        return state;
    }

    public void flipAlliance() {
        states.replaceAll(TrajectoryState::flipped);
    }

    public boolean completed() {
        return step >= states.size();
    }
}

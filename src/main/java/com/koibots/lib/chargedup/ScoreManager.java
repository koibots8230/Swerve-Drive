package com.koibots.lib.chargedup;

import edu.wpi.first.math.geometry.Pose2d;
import lombok.Getter;
import org.ejml.simple.SimpleMatrix;

public class ScoreManager {
    public enum GamePiece {
        CUBE,
        CONE
    }

    @Getter
    public static final ScoreManager instance = new ScoreManager();

    int score = 0;
    SimpleMatrix grid = new SimpleMatrix(3, 9);

    ScoreManager() {
        grid.zero();
    }

    public void registerPiece(int row, int column) {
        grid.set(row, column, grid.get(row, column) + 1);
    }

    public double[] getOptimalPlacement(Pose2d pos, GamePiece piece) {
        // TODO: implement this

        return new double[] {0, 0};
    }
}
